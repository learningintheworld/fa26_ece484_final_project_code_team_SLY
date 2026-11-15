"""Image-domain lane evidence. No vehicle geometry, metres, or control output.

The sliding-window idea is related to ECE484 MP1 line_fit.py, but candidates,
association and corridor selection are independent; no MP module is imported.
"""
import time
import cv2
import numpy as np


DEFAULTS = dict(roi_top=0.54, row_step=6, gray_min=60, saturation_max=100,
                contrast_min=22, stripe_width_max=24, track_gate=30,
                max_gap_rows=3, candidate_span=65, min_samples=9,
                fit_tolerance=5, ransac_trials=35, near_row_min=0.73,
                common_span=45, width_min=65, width_max_fraction=0.95,
                anchor_margin=12, ambiguity_margin=0.10,
                association_distance=28, reset_gap_s=0.5, switch_frames=4,
                memory_s=1.0, init_frames=1, duplicate_distance=12, fit_mode="quadratic", paint_color="white", anchor_fraction=None)


def camera_maps(camera, shape):
    h, w = shape[:2]
    if (camera.get('width'), camera.get('height')) != (w, h):
        raise ValueError('CAMERA_INFO_SIZE_MISMATCH')
    k = np.asarray(camera['K'], dtype=float).reshape(3, 3)
    d = np.asarray(camera['D'], dtype=float)
    model = camera.get('distortion_model')
    if model not in ('plumb_bob', 'rational_polynomial'):
        raise ValueError('UNSUPPORTED_DISTORTION_MODEL')
    if d.size not in (4, 5, 8, 12, 14) or (model == 'rational_polynomial' and d.size < 8):
        raise ValueError('INVALID_DISTORTION_COEFFICIENTS')
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('INVALID_CAMERA_INTRINSICS')
    if not (0 <= k[0, 2] < w and 0 <= k[1, 2] < h) or not np.allclose(k[2], [0, 0, 1]):
        raise ValueError('INVALID_CAMERA_INTRINSICS')
    maps = cv2.initUndistortRectifyMap(k, d, None, k, (w, h), cv2.CV_32FC1)
    return maps, float(k[0, 2])


def curve_x(candidate, rows):
    if 'knots' in candidate:
        points=np.asarray(candidate['knots'])
        return np.interp(rows,points[:,1],points[:,0])
    return np.polyval(candidate['coef'], np.asarray(rows) / candidate['height'])


class LaneDetector:
    def __init__(self, config=None):
        self.cfg = {**DEFAULTS, **(config or {})}
        self.map_key = self.maps = None
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        self.reset()

    def reset(self):
        self.previous = []
        self.gray_previous = None
        self.roles = {}
        self.memory = {}
        self.identity_lost = False
        self.role_seen_stamp = None
        self.previous_pair = None
        self.last_stamp = self.last_valid_stamp = None
        self.next_id = 1
        self.pending_pair = None
        self.pending_count = 0

    def mask(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        saturation = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 1]
        enhanced = self.clahe.apply(gray)
        contrast = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT,
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
        edges = cv2.dilate(cv2.Canny(enhanced, 35, 90), np.ones((5, 5), np.uint8))
        white = saturation <= self.cfg['saturation_max']
        if self.cfg['paint_color'] == 'yellow':
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            white = (hsv[:, :, 0] >= 15) & (hsv[:, :, 0] <= 40) & (saturation >= 70)
        mask = ((gray >= self.cfg['gray_min']) & white &
                (contrast >= self.cfg['contrast_min']) & (edges > 0)).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        mask[:int(image.shape[0] * self.cfg['roi_top'])] = 0
        # Remap borders are not paint.
        valid = (self.maps[0] >= 2) & (self.maps[0] < image.shape[1]-2) & (self.maps[1] >= 2) & (self.maps[1] < image.shape[0]-2)
        mask[~valid] = 0
        return mask

    def candidates(self, mask):
        h, w = mask.shape
        tracks = []
        step = int(self.cfg['row_step'])
        for y in range(h - 5, int(h * self.cfg['roi_top']), -step):
            active = np.count_nonzero(mask[max(0, y-1):y+2], axis=0) >= 2
            changes = np.diff(np.r_[False, active, False].astype(np.int8))
            runs = [(float((a+b-1)/2), b-a) for a, b in zip(np.flatnonzero(changes==1), np.flatnonzero(changes==-1))
                    if 2 <= b-a <= self.cfg['stripe_width_max']]
            choices = []
            for ti, track in enumerate(tracks):
                last_x, last_y = track[-1]
                dy = last_y-y
                if dy > step*(self.cfg['max_gap_rows']+1): continue
                slope = 0.0
                if len(track) >= 2:
                    older = track[max(0, len(track)-4)]
                    slope = np.clip((last_x-older[0])/(last_y-older[1]), -4.5, 4.5)
                predicted = last_x-slope*dy
                for ri, (x, _) in enumerate(runs):
                    error = abs(x-predicted)
                    if error <= self.cfg['track_gate'] * (1 + 0.25*(dy/step-1)):
                        choices.append((error, ti, ri))
            used_t, used_r = set(), set()
            for _, ti, ri in sorted(choices):
                if ti in used_t or ri in used_r: continue
                tracks[ti].append((runs[ri][0], y)); used_t.add(ti); used_r.add(ri)
            tracks.extend([[(x, y)] for ri, (x, _) in enumerate(runs) if ri not in used_r])
        output = []
        rng = np.random.default_rng(7)
        for track in tracks:
            pts = np.asarray(track, float)
            if len(pts) < self.cfg['min_samples'] or np.ptp(pts[:, 1]) < self.cfg['candidate_span']: continue
            if self.cfg['fit_mode']=='piecewise':
                pts=pts[np.argsort(pts[:,1])]
                # Row association already rejects disconnected candidates;
                # retain the observed bend instead of forcing one quadratic.
                smoothed=pts.copy()
                for k in range(1,len(pts)-1):smoothed[k,0]=np.median(pts[k-1:k+2,0])
                output.append(dict(coef=np.polyfit(pts[:,1]/h,pts[:,0],2).tolist(),height=h,
                    y_min=float(pts[0,1]),y_max=float(pts[-1,1]),support=len(pts),residual_px=0.,
                    score=float(min(1.,len(pts)*step/(np.ptp(pts[:,1])+step))),knots=smoothed.tolist()))
                continue
            yn = pts[:, 1]/h
            design = np.column_stack((yn*yn, yn, np.ones_like(yn)))
            best = np.zeros(len(pts), bool)
            for _ in range(self.cfg['ransac_trials']):
                subset = rng.choice(len(pts), 3, replace=False)
                try: coef = np.linalg.solve(design[subset], pts[subset, 0])
                except np.linalg.LinAlgError: continue
                good = np.abs(design@coef-pts[:, 0]) < self.cfg['fit_tolerance']
                if good.sum() > best.sum(): best = good
            if best.sum() < self.cfg['min_samples'] or best.mean() < .65: continue
            pts = pts[best]
            lo, hi = float(pts[:, 1].min()), float(pts[:, 1].max())
            if hi-lo < self.cfg['candidate_span']: continue
            coef = np.linalg.lstsq(design[best], pts[:, 0], rcond=None)[0]
            residual = float(np.median(np.abs(design[best]@coef-pts[:, 0])))
            output.append(dict(coef=coef.tolist(), height=h, y_min=lo, y_max=hi,
                               support=int(best.sum()), residual_px=residual,
                               score=float(min(1., best.sum()*step/(hi-lo+step)))))
        # Merge duplicate fits of the same stripe before pairing.
        output.sort(key=lambda c: (-c['support'], c['residual_px']))
        unique = []
        for c in output:
            duplicate = False
            for other in unique:
                lo, hi = max(c['y_min'], other['y_min']), min(c['y_max'], other['y_max'])
                if hi-lo >= self.cfg['common_span']:
                    ys = np.linspace(lo, hi, 12)
                    if np.median(np.abs(curve_x(c, ys)-curve_x(other, ys))) < self.cfg['duplicate_distance']:
                        duplicate = True; break
            if not duplicate: unique.append(c)
        return self.join_fragments(unique)[:16]

    def join_fragments(self, candidates):
        # Connect pieces of the SAME stripe at adjacent endpoints, not across
        # parallel lanes. Competing continuations are deliberately left apart.
        candidates=list(candidates)
        while True:
            links=[]
            for i,a in enumerate(candidates):
                for j,b in enumerate(candidates):
                    if i==j or not a['y_max']<b['y_max']:continue
                    gap=b['y_min']-a['y_max']
                    if not -44<=gap<=12:continue
                    y=(a['y_max']+b['y_min'])/2
                    error=abs(float(curve_x(a,y)-curve_x(b,y)))
                    sa=float((curve_x(a,a['y_max'])-curve_x(a,a['y_max']-6))/6)
                    sb=float((curve_x(b,b['y_min']+6)-curve_x(b,b['y_min']))/6)
                    if error<12 and abs(sa-sb)<1.2:links.append((i,j))
            unique=[(i,j) for i,j in links if sum(i in pair for pair in links)==1 and sum(j in pair for pair in links)==1]
            if not unique:break
            i,j=unique[0];a,b=candidates[i],candidates[j]
            ys=np.unique(np.r_[np.linspace(a['y_min'],a['y_max'],32),np.linspace(b['y_min'],b['y_max'],32)])
            xs=[]
            for y in ys:
                present=[c for c in (a,b) if c['y_min']<=y<=c['y_max']]
                xs.append(float(np.mean([curve_x(c,y) for c in present])))
            combined=dict(a,knots=np.column_stack((xs,ys)).tolist(),y_min=float(ys[0]),y_max=float(ys[-1]),
                          support=a['support']+b['support'],score=min(a['score'],b['score']),
                          residual_px=max(a['residual_px'],b['residual_px']))
            candidates=[c for k,c in enumerate(candidates) if k not in (i,j)]+[combined]
        return candidates

    def associate(self, candidates, gray=None, stamp=None):
        # Optical flow only predicts association. A prediction never becomes
        # an observed boundary or a valid centreline by itself.
        pool = self.memory if stamp is not None else {p['id']: (p, 0) for p in self.previous}
        predictions = []
        for identity, (old, seen) in list(pool.items()):
            if stamp is not None and (stamp-seen)/1e9 > self.cfg['memory_s']:
                continue
            predicted = dict(old)
            if gray is not None and self.gray_previous is not None:
                rows = np.linspace(old['y_min'], old['y_max'], 24)
                pts = np.column_stack((curve_x(old, rows), rows)).astype(np.float32).reshape(-1,1,2)
                nxt, ok, _ = cv2.calcOpticalFlowPyrLK(self.gray_previous, gray, pts, None,
                    winSize=(21,21), maxLevel=3)
                if nxt is not None:
                    back, ok2, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray_previous, nxt, None,
                        winSize=(21,21), maxLevel=3)
                    good = (ok.ravel()>0) & (ok2.ravel()>0) & (np.linalg.norm(back-pts,axis=2).ravel()<1.5)
                    xy = nxt.reshape(-1,2)[good]
                    if len(xy)>=8 and np.ptp(xy[:,1])>30:
                        if 'knots' in old:
                            predicted['knots']=xy[np.argsort(xy[:,1])].tolist()
                        predicted.update(coef=np.polyfit(xy[:,1]/old['height'],xy[:,0],2).tolist(),
                                         y_min=float(xy[:,1].min()), y_max=float(xy[:,1].max()))
            predictions.append((identity,predicted,seen))
        matches = []
        for i,c in enumerate(candidates):
            for identity,p,seen in predictions:
                lo,hi=max(c['y_min'],p['y_min']),min(c['y_max'],p['y_max'])
                if hi-lo<30: continue
                rows=np.linspace(lo,hi,10)
                error=float(np.median(np.abs(curve_x(c,rows)-curve_x(p,rows))))
                # Paint has an aperture ambiguity: LK can slide along a stripe
                # while passing forward/backward consistency. Also compare the
                # actual previous observation, rather than trusting LK alone.
                observed=pool[identity][0]
                direct_lo,direct_hi=max(c['y_min'],observed['y_min']),min(c['y_max'],observed['y_max'])
                if direct_hi-direct_lo>=30:
                    direct_rows=np.linspace(direct_lo,direct_hi,10)
                    error=min(error,float(np.median(np.abs(curve_x(c,direct_rows)-curve_x(observed,direct_rows)))))
                if error<self.cfg['association_distance']:
                    overlap=(hi-lo)/max(c['y_max']-c['y_min'],p['y_max']-p['y_min'])
                    matches.append((0 if identity in self.roles else 1,error+20*(1-overlap),i,identity))
        used=set()
        for _,_,i,identity in sorted(matches):
            if 'id' not in candidates[i] and identity not in used:
                candidates[i]['id']=identity; used.add(identity)
        for c in candidates:
            if 'id' not in c: c['id']=self.next_id; self.next_id+=1
        if stamp is not None:
            self.memory={identity:(p,seen) for identity,p,seen in predictions}
            self.memory.update({c['id']:(dict(c),stamp) for c in candidates})
            if any(c['id'] in self.roles for c in candidates): self.role_seen_stamp=stamp
            if self.roles and self.role_seen_stamp is not None and (stamp-self.role_seen_stamp)/1e9>self.cfg['memory_s']:
                self.identity_lost=True
        self.gray_previous=gray

    def select(self, candidates, width, height, anchor):
        pairs, rejected = [], []
        if self.identity_lost:
            return None, "CORRIDOR_LOST_REARM_REQUIRED", pairs, rejected
        for i, a in enumerate(candidates):
            for b in candidates[i+1:]:
                lo, hi = max(a['y_min'], b['y_min']), min(a['y_max'], b['y_max'])
                if hi-lo < self.cfg['common_span'] or hi < height*self.cfg['near_row_min']: continue
                ys = np.linspace(lo, hi, 24)
                if curve_x(a, hi) > curve_x(b, hi): left, right = b, a
                else: left, right = a, b
                lx, rx = curve_x(left, ys), curve_x(right, ys)
                widths = rx-lx
                reason = None
                if np.any(widths < 20): reason = 'CROSSING_OR_THIN_PAIR'
                elif widths[-1] < self.cfg['width_min'] or widths[-1] > width*self.cfg['width_max_fraction']: reason = 'WIDTH_INVALID'
                elif not self.roles and not lx[-1]+self.cfg['anchor_margin'] < anchor < rx[-1]-self.cfg['anchor_margin']: reason = 'NOT_EGO_CORRIDOR'
                elif widths[0] > widths[-1]*1.25: reason = 'WIDTH_PERSPECTIVE_INCONSISTENT'
                elif np.any(lx < -5) or np.any(rx > width+5): reason = 'OUTSIDE_IMAGE'
                if reason is None and (self.roles.get(left['id']) == 'right' or self.roles.get(right['id']) == 'left'):
                    reason = 'BOUNDARY_ROLE_CONFLICT'
                if reason is None and self.roles and left['id'] not in self.roles and right['id'] not in self.roles:
                    reason = 'CORRIDOR_IDENTITY_UNSUPPORTED'
                if reason is None:
                    for middle in candidates:
                        if middle is left or middle is right: continue
                        overlap = (ys >= middle['y_min']) & (ys <= middle['y_max'])
                        if overlap.sum() < .70*len(ys): continue
                        mx = curve_x(middle, ys[overlap])
                        if np.mean((mx>lx[overlap]+12)&(mx<rx[overlap]-12)) > .75:
                            reason = 'SPANS_ANOTHER_BOUNDARY'; break
                ids = (left['id'], right['id'])
                if reason:
                    rejected.append(dict(ids=list(ids), reason=reason)); continue
                support = (left['score']+right['score'])/2
                span = min(1., (hi-lo)/150)
                center_bias = abs((lx[-1]+rx[-1])/2-anchor)/(widths[-1]/2)
                score = .45*support+.35*span+.20*(1-center_bias)
                base_score = score
                if ids == self.previous_pair: score += .15
                pairs.append(dict(ids=list(ids), score=float(score), base_score=float(base_score), y_min=lo, y_max=hi,
                                  left=np.column_stack((lx,ys)).tolist(), right=np.column_stack((rx,ys)).tolist(),
                                  center=np.column_stack(((lx+rx)/2,ys)).tolist()))
        pairs.sort(key=lambda p:p['score'], reverse=True)
        if not pairs:
            self.pending_pair, self.pending_count = None, 0
            return None, 'NO_SUPPORTED_EGO_PAIR', pairs, rejected
        unboosted = sorted(pairs, key=lambda p:p['base_score'], reverse=True)
        if len(pairs)>1 and unboosted[0]['base_score']-unboosted[1]['base_score'] < self.cfg['ambiguity_margin']:
            self.pending_pair, self.pending_count = None, 0
            return None, 'AMBIGUOUS_CORRIDOR', pairs, rejected
        chosen = pairs[0]; ids = tuple(chosen['ids'])
        if self.previous_pair is None and self.cfg['init_frames'] > 1:
            if ids == self.pending_pair: self.pending_count += 1
            else: self.pending_pair, self.pending_count = ids, 1
            if self.pending_count < self.cfg['init_frames']:
                return None, 'INITIALIZING_CORRIDOR', pairs, rejected
        if self.previous_pair is not None and ids != self.previous_pair:
            if ids == self.pending_pair: self.pending_count += 1
            else: self.pending_pair, self.pending_count = ids, 1
            if self.pending_count < self.cfg['switch_frames']:
                return None, 'REACQUIRING_CORRIDOR', pairs, rejected
        self.pending_pair, self.pending_count = None, 0
        return chosen, 'VISION_VALID_PIXELS_ONLY', pairs, rejected

    def process(self, image_bgr, camera, stamp_ns):
        start = time.perf_counter()
        result = dict(stamp_ns=int(stamp_ns), vision_valid=False, metric_valid=False,
                      metric_reason='NOT_USED_IMAGE_SERVO', coordinate_space='undistorted_image_pixels',
                      candidates=[], selected=[], left=[], right=[], center=[], confidence=0., pairs=[], rejected_pairs=[])
        debug = image_bgr.copy()
        try:
            if image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or image_bgr.dtype != np.uint8:
                raise ValueError('INVALID_IMAGE')
            if self.last_stamp is not None and (stamp_ns<=self.last_stamp or (stamp_ns-self.last_stamp)/1e9>self.cfg['reset_gap_s']):
                self.reset()
            self.last_stamp = stamp_ns
            key = (camera['width'],camera['height'],tuple(camera['K']),tuple(camera['D']),camera['distortion_model'])
            if key != self.map_key:
                self.maps, self.anchor = camera_maps(camera, image_bgr.shape)
                self.map_key = key
                self.previous, self.previous_pair = [], None
            elif image_bgr.shape[:2] != (camera['height'], camera['width']):
                raise ValueError('CAMERA_INFO_SIZE_MISMATCH')
            debug = cv2.remap(image_bgr, *self.maps, cv2.INTER_LINEAR)
            mask = self.mask(debug)
            candidates = self.candidates(mask)
            self.associate(candidates, cv2.cvtColor(debug,cv2.COLOR_BGR2GRAY), stamp_ns)
            h,w = image_bgr.shape[:2]
            anchor = self.anchor if self.cfg['anchor_fraction'] is None else w*self.cfg['anchor_fraction']
            chosen, reason, pairs, rejected = self.select(candidates,w,h,anchor)
            for c in candidates:
                rows = np.linspace(c['y_min'],c['y_max'],32)
                c['points'] = np.column_stack((curve_x(c,rows),rows)).tolist()
            result.update(candidates=candidates, reason=reason, pairs=pairs, rejected_pairs=rejected)
            if chosen:
                result.update(vision_valid=True,selected=chosen['ids'],left=chosen['left'],right=chosen['right'],
                              center=chosen['center'],confidence=min(1.,chosen['score']))
                self.previous_pair=tuple(chosen['ids']);self.last_valid_stamp=stamp_ns
                self.roles[chosen['ids'][0]]='left'; self.roles[chosen['ids'][1]]='right'
                self.role_seen_stamp=stamp_ns
            self.previous = candidates
        except (ValueError,KeyError,TypeError,cv2.error) as exc:
            result['reason'] = str(exc);self.reset()
        result['processing_ms'] = (time.perf_counter()-start)*1000
        result['debug_image'] = render(debug,result)
        return result


def render(image, result):
    output = image.copy()
    for c in result['candidates']:
        pts=np.rint(c['points']).astype(np.int32)
        cv2.polylines(output,[pts],False,(190,160,80),1,cv2.LINE_AA)
        p=tuple(pts[-1]);cv2.putText(output,str(c['id']),p,cv2.FONT_HERSHEY_SIMPLEX,.4,(255,200,0),1)
    if result['vision_valid']:
        for name,color in [('left',(30,210,30)),('right',(0,170,255)),('center',(255,0,255))]:
            cv2.polylines(output,[np.rint(result[name]).astype(np.int32)],False,color,2,cv2.LINE_AA)
    cv2.rectangle(output,(0,0),(output.shape[1],66),(25,25,25),-1)
    cv2.putText(output,result['reason'],(10,22),cv2.FONT_HERSHEY_SIMPLEX,.52,(220,240,220),1,cv2.LINE_AA)
    cv2.putText(output,'PIXELS ONLY | NOT_USED_IMAGE_SERVO | score %.2f'%result['confidence'],(10,45),cv2.FONT_HERSHEY_SIMPLEX,.48,(220,220,220),1,cv2.LINE_AA)
    return output
