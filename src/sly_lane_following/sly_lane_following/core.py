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
        backend = self.cfg.get('mask_backend', 'opencv')
        if backend not in ('opencv', 'enet'):
            raise ValueError('UNKNOWN_MASK_BACKEND')
        self.neural_mask = None
        if backend == 'enet':
            from .neural import NeuralMask
            self.neural_mask = NeuralMask(self.cfg['enet_checkpoint'], self.cfg.get('enet_device', 'cuda'))
        self.reset()

    def reset(self):
        self.previous = []
        self.gray_previous = None
        self.roles = {}
        self.memory = {}
        self.observation_history = {}
        self.identity_lost = False
        self.fragment_links = {}
        self.role_seen_stamp = None
        self.previous_pair = None
        self.last_stamp = self.last_valid_stamp = None
        self.next_id = 1
        self.pending_pair = None
        self.pending_count = 0

    def mask(self, image):
        neural_prediction = None
        if self.neural_mask is not None:
            mask = self.neural_mask(image)
            mask[:int(image.shape[0] * self.cfg['roi_top'])] = 0
            valid = (self.maps[0] >= 2) & (self.maps[0] < image.shape[1]-2) & (self.maps[1] >= 2) & (self.maps[1] < image.shape[0]-2)
            mask[~valid] = 0
            if not self.cfg.get('enet_refine_paint', False):
                return mask
            # Refine learned lane regions with current RGB paint evidence.
            # Intersection only: classic processing cannot fill a neural miss.
            neural_prediction = mask
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
        if neural_prediction is None:
            return mask
        mask = cv2.bitwise_and(mask, neural_prediction)
        # ENet already supplies semantic support. Keep the INTERIOR of a
        # narrow observed paint stripe; Canny alone splits thick paint into
        # two false boundaries. Wide transverse bends keep the edge scan.
        interior = ((neural_prediction > 0) & white & valid &
                    (gray >= self.cfg['gray_min']))
        interior[:int(image.shape[0] * self.cfg['roi_top'])] = False
        for y in np.flatnonzero(interior.any(axis=1)):
            changes = np.diff(np.r_[False, interior[y], False].astype(np.int8))
            for lo, hi in zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)):
                if hi-lo <= self.cfg['stripe_width_max']:
                    mask[y, lo:hi] = 255
        return mask

    def row_identity_priority(self, track):
        """Prefer the CURRENT near fragment of a recently confirmed boundary.

        Only breaks competition for an already detected row run. It does not
        add pixels or extend a track across the existing spatial gate.
        """
        if not self.cfg.get('preserve_row_identity', False) or self.last_stamp is None:
            return 1
        near = np.asarray(track[:10], float)
        if len(near) < 5 or np.ptp(near[:, 1]) < 24:
            return 1
        matched = []
        for identity in self.previous_pair or ():
            observations = self.observation_history.get(identity, [])
            if not observations and identity in self.memory:
                observations = [self.memory[identity]]
            for old, seen in reversed(observations):
                if not 0 < (self.last_stamp-seen)/1e9 <= .25:
                    continue
                inside = (near[:,1]>=old['y_min']) & (near[:,1]<=old['y_max'])
                q = near[inside]
                if len(q)>=5 and np.ptp(q[:,1])>=24 and np.percentile(abs(q[:,0]-curve_x(old,q[:,1])),90)<12:
                    matched.append(identity)
                    break
        return 0 if len(matched)==1 else 1

    def candidates(self, mask):
        h, w = mask.shape
        tracks = []
        near_priorities = {}
        step = int(self.cfg['row_step'])
        for y in range(h - 5, int(h * self.cfg['roi_top']), -step):
            active = np.count_nonzero(mask[max(0, y-1):y+2], axis=0) >= 2
            changes = np.diff(np.r_[False, active, False].astype(np.int8))
            runs = [(float((a+b-1)/2), b-a) for a, b in zip(np.flatnonzero(changes==1), np.flatnonzero(changes==-1))
                    if 2 <= b-a <= self.cfg['stripe_width_max']
                    # In the dual-scan profile, an edge-clipped cross-section
                    # is not a stripe centre. The other scan may still see it.
                    and (not self.cfg.get('column_scan', False) or (a > 0 and b < w))]
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
                if ti in near_priorities:
                    priority = near_priorities[ti]
                else:
                    priority = self.row_identity_priority(track)
                    # The first ten observed points never change as a track
                    # extends upward; avoid rechecking them at every row.
                    if len(track)>=10:
                        near_priorities[ti] = priority
                for ri, (x, _) in enumerate(runs):
                    error = abs(x-predicted)
                    if error <= self.cfg['track_gate'] * (1 + 0.25*(dy/step-1)):
                        choices.append((priority, error, ti, ri))
            used_t, used_r = set(), set()
            for _, _, ti, ri in sorted(choices):
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
        # Near-horizontal real paint is wide in a row but thin in a column.
        # Opt-in only: the qualified simulator keeps its original row scan.
        if self.cfg.get('column_scan', False):
            output.extend(self.column_candidates(mask))
        if self.cfg.get("split_unsupported_connections", False):
            output = self.split_mask_gaps(output, mask)
        if self.cfg.get("merge_observed_overlap", False):
            output = self.merge_observed_overlap(output, mask)
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
        joined = self.join_fragments(unique, mask if self.cfg.get('column_scan',False) else None)
        if self.cfg.get('split_unsupported_connections', False):
            joined = self.split_mask_gaps(joined, mask)
        if self.cfg.get("confirm_mask_support", False):
            self.confirm_mask_support(joined, mask)
        return self.recover_tracked_mask(joined,mask)[:16]

    def recover_tracked_mask(self, candidates, mask):
        """Recover only a uniquely matched, CURRENT connected paint section.

        Short near paint and a curved row/column split can otherwise disappear
        from candidates despite a good mask. No pixels cross a gap or branch.
        The real fragment profile opts in; normal/simulator profiles do not.
        """
        if not self.cfg.get('fragment_identity_handoff',False) or not self.roles or self.last_stamp is None:
            return candidates
        h,w=mask.shape;_,labels,stats,_=cv2.connectedComponentsWithStats((mask>0).astype('uint8'),8)
        recovered=[]
        for label,stat in enumerate(stats[1:],1):
            if stat[cv2.CC_STAT_AREA]<80 or stat[cv2.CC_STAT_HEIGHT]<24:continue
            points=[];cross_sections=[]
            for y in range(int(stat[1]+stat[3]-1),int(stat[1])-1,-1):
                xs=np.flatnonzero(labels[y]==label)
                if not len(xs):break
                runs=np.split(xs,np.flatnonzero(np.diff(xs)>1)+1)
                if len(runs)!=1:break
                a,b=runs[0][0],runs[0][-1]+1
                if a<=0 or b>=w or b-a<2:
                    if not points:continue
                    break
                points.append(((a+b-1)/2,y));cross_sections.append(b-a)
            if len(points)<24:continue
            # A slanted stripe is wider in a horizontal scan. Validate its
            # perpendicular thickness rather than deleting a good shallow arc.
            q=np.asarray(points);slope=np.gradient(q[:,0],q[:,1])
            thickness=np.asarray(cross_sections)/np.sqrt(1+slope*slope)
            stop=next((i for i,v in enumerate(thickness) if v>self.cfg['stripe_width_max']),len(q))
            p=q[:stop][::-1]
            if len(p)<24:continue
            span=np.ptp(p[:,1]);arc=np.linalg.norm(np.diff(p,axis=0),axis=1).sum()
            if span<24 or arc<40:continue
            matching=[]
            for identity in self.roles:
                item=self.memory.get(identity)
                if item is None or not 0<(self.last_stamp-item[1])/1e9<=.15:continue
                old=item[0];hi=min(old['y_max'],p[-1,1]);lo=max(old['y_min'],p[0,1],hi-36.)
                if hi-lo<24:continue
                y=np.linspace(lo,hi,12)
                if np.percentile(abs(np.interp(y,p[:,1],p[:,0])-curve_x(old,y)),90)<12:
                    matching.append(identity)
            if len(matching)!=1:continue
            duplicate=[];covered=False
            for j,c in enumerate(candidates):
                lo=max(p[0,1],c['y_min']);hi=min(p[-1,1],c['y_max'])
                if hi-lo<24:continue
                y=np.linspace(lo,hi,20)
                if np.percentile(abs(np.interp(y,p[:,1],p[:,0])-curve_x(c,y)),90)>8:continue
                if c['y_min']<=p[0,1]+8 and c['y_max']>=p[-1,1]-8:covered=True;break
                if c['y_min']>=p[0,1]-2 and c['y_max']<=p[-1,1]+2:duplicate.append(j)
            if covered:
                # A barely 30-row remnant can lose the ordinary 30-row match
                # after LK slides along paint. The current full-section match
                # above supplies a stricter, unique near identity instead.
                if c['y_max']-c['y_min']<36:
                    recovered.append(([j],dict(c,mask_anchor_id=matching[0])))
                continue
            verified=self.fragment_links.get(matching[0],{}).get('count',0)>=3
            # A current near remnant can be shorter than the generic 80-pixel
            # candidate arc without needing a far-fragment handoff. It already
            # passed full cross sections, >=24 shared rows, <12 px residual,
            # unique identity, >=40 px arc and <=0.15 s actual-observation age.
            near_remnant = span<self.cfg['candidate_span'] and p[-1,1]>=.85*h
            if len(duplicate)<2 and not (verified and span<self.cfg['candidate_span']) and not near_remnant:continue
            recovered.append((duplicate,dict(coef=np.polyfit(p[:,1]/h,p[:,0],2).tolist(),height=h,
                y_min=float(p[0,1]),y_max=float(p[-1,1]),support=len(p),residual_px=1.,score=1.,
                knots=p.tolist(),tracked_mask_recovery=True,mask_anchor_id=matching[0])))
        # A role must not seed multiple competing current components.
        counts={i:sum(c['mask_anchor_id']==i for _,c in recovered) for i in self.roles}
        kept=[(indices,c) for indices,c in recovered if counts[c['mask_anchor_id']]==1]
        removed={j for indices,_ in kept for j in indices}
        return [c for j,c in enumerate(candidates) if j not in removed]+[c for _,c in kept]

    @staticmethod
    def confirm_mask_support(candidates, mask):
        """Confirm sparse row samples using dense current-frame paint evidence.

        No curve extension or gap filling. One-pixel tolerance handles raster
        rounding; missing stretches and unsupported neighbouring lines fail.
        """
        h, w = mask.shape
        visible = cv2.dilate(mask, np.ones((3, 3), np.uint8))
        for c in candidates:
            if c['score'] >= .7:
                continue
            ys = np.linspace(c['y_min'], c['y_max'],
                             max(2, int(2*(c['y_max']-c['y_min']))+1))
            points = np.column_stack((curve_x(c, ys), ys))
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
            if not np.isfinite(arc).all() or arc[-1] < 80:
                continue
            at = np.arange(0., arc[-1], 1.)
            xy = np.rint(np.column_stack((np.interp(at, arc, points[:, 0]),
                                         np.interp(at, arc, points[:, 1])))).astype(int)
            inside = (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
            supported = np.zeros(len(xy), bool)
            supported[inside] = visible[xy[inside, 1], xy[inside, 0]] > 0
            changes = np.diff(np.r_[False, ~supported, False].astype(int))
            gaps = np.flatnonzero(changes == -1)-np.flatnonzero(changes == 1)
            fraction = float(supported.mean())
            gap = int(gaps.max()) if len(gaps) else 0
            c['mask_support_fraction'] = fraction
            c['mask_unsupported_run_px'] = gap
            if fraction >= .98 and gap <= 3:
                c['row_sample_score'] = c['score']
                c['score'] = fraction

    def column_candidates(self, mask):
        """Recover observed shallow arcs; split folds instead of extrapolating.

        Column support complements row support. Returned curves retain the
        existing x(y) interface, so only monotone-y pieces are admissible.
        """
        h, w = mask.shape; step = int(self.cfg['row_step']); tracks = []
        for x in range(4, w-4, step):
            active = np.count_nonzero(mask[:, x-1:x+2], axis=1) >= 2
            changes = np.diff(np.r_[False, active, False].astype(np.int8))
            runs = [(float((a+b-1)/2), b-a) for a,b in
                    zip(np.flatnonzero(changes==1),np.flatnonzero(changes==-1))
                    if 2 <= b-a <= self.cfg['stripe_width_max'] and a > 0 and b < h]
            choices=[]
            for ti,track in enumerate(tracks):
                last_x,last_y=track[-1];dx=x-last_x
                if dx>step*(self.cfg['max_gap_rows']+1):continue
                slope=0.
                if len(track)>=2:
                    old=track[max(0,len(track)-4)]
                    slope=np.clip((last_y-old[1])/(last_x-old[0]),-2.,2.)
                for ri,(y,_) in enumerate(runs):
                    error=abs(y-(last_y+slope*dx))
                    if error<=12*(1+.25*(dx/step-1)):choices.append((error,ti,ri))
            used_t=set();used_r=set()
            for _,ti,ri in sorted(choices):
                if ti not in used_t and ri not in used_r:
                    tracks[ti].append((x,runs[ri][0]));used_t.add(ti);used_r.add(ri)
            tracks.extend([[(x,y)] for ri,(y,_) in enumerate(runs) if ri not in used_r])
        output=[]
        for track in tracks:
            pts=np.asarray(track,float)
            if len(pts)<12:continue
            # Smooth pixel-scale jitter, never bridge a missing observation.
            smooth=pts.copy()
            for i in range(1,len(pts)-1):smooth[i,1]=np.median(pts[i-1:i+2,1])
            cuts=[0];direction=0
            for i,dy in enumerate(np.diff(smooth[:,1])):
                if abs(dy)<.75:continue
                sign=int(np.sign(dy))
                if direction and sign!=direction:cuts.append(i)
                direction=sign
            cuts.append(len(smooth)-1)
            for a,b in zip(cuts[:-1],cuts[1:]):
                p=smooth[a:b+1]
                if len(p)<12 or np.ptp(p[:,1])<self.cfg.get('column_span',24.):continue
                arc=np.linalg.norm(np.diff(p,axis=0),axis=1).sum()
                if arc<self.cfg.get('candidate_arc',80.):continue
                if p[0,1]>p[-1,1]:p=p[::-1]
                # Repeated image rows cannot describe two different x's.
                rows=np.unique(p[:,1]);knots=np.array([[np.median(p[p[:,1]==y,0]),y] for y in rows])
                if len(knots)<6:continue
                output.append(dict(coef=np.polyfit(knots[:,1]/h,knots[:,0],2).tolist(),height=h,
                    y_min=float(rows[0]),y_max=float(rows[-1]),support=len(p),residual_px=0.,
                    score=float(min(1.,len(p)*step/(np.ptp(p[:,0])+step))),
                    knots=knots.tolist(),scan_axis='column'))
        return output

    def split_mask_gaps(self, candidates, mask):
        """Reject long unobserved connectors before they can inherit an identity.

        Row/column tracking may jump between two different stripes. Inspect
        the fitted curve in arc length, not just row spacing; keep observed
        fragments and never fill a missing stripe with a straight connector.
        """
        supported = cv2.dilate(mask, np.ones((3, 3), np.uint8))
        output = []
        for c in candidates:
            rows = np.linspace(c['y_min'], c['y_max'], max(8, int(2*(c['y_max']-c['y_min']))+1))
            xy = np.column_stack((curve_x(c, rows), rows))
            arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
            if arc[-1] < 1: continue
            distance = np.linspace(0, arc[-1], int(np.ceil(arc[-1]))+1)
            points = np.column_stack([np.interp(distance, arc, xy[:, k]) for k in (0, 1)])
            pixels = np.rint(points).astype(int)
            inside = ((pixels[:, 0]>=0)&(pixels[:, 0]<mask.shape[1])&
                      (pixels[:, 1]>=0)&(pixels[:, 1]<mask.shape[0]))
            seen = np.zeros(len(points), bool)
            seen[inside] = supported[pixels[inside, 1], pixels[inside, 0]] > 0
            changes = np.diff(np.r_[False, ~seen, False].astype(np.int8))
            gaps = [(a,b) for a,b in zip(np.flatnonzero(changes==1), np.flatnonzero(changes==-1))
                    if distance[b-1]-distance[a] > 12.]
            if not gaps:
                output.append(c); continue
            start = 0
            for a,b in gaps+[(len(points),len(points))]:
                fragment = points[start:a]; start = b
                if len(fragment)<2: continue
                min_span = self.cfg.get('column_span',24.) if c.get('scan_axis')=='column' else self.cfg['candidate_span']
                if np.ptp(fragment[:, 1])<min_span: continue
                if np.linalg.norm(np.diff(fragment, axis=0), axis=1).sum()<self.cfg.get('candidate_arc',80.): continue
                output.append(dict(c, knots=fragment.tolist(), y_min=float(fragment[0,1]),
                                   y_max=float(fragment[-1,1]), split_unobserved_connector=True))
        return output

    def merge_observed_overlap(self, candidates, mask):
        """Unify two scans of the same observed paint, never choose a fork.

        Compare tangents at the SAME image rows. Comparing opposite endpoints
        confuses a curved stripe with two incompatible straight fragments.
        Every merge needs a long agreement region and paint between the fits.
        """
        candidates = list(candidates)
        while True:
            links = []
            for i, a in enumerate(candidates):
                for j in range(i + 1, len(candidates)):
                    b = candidates[j]
                    lo, hi = max(a['y_min'], b['y_min']), min(a['y_max'], b['y_max'])
                    span = min(a['y_max'] - a['y_min'], b['y_max'] - b['y_min'])
                    if hi - lo < 24 or hi - lo < .45 * span: continue
                    if min(a['score'], b['score']) < .7: continue
                    rows = np.linspace(lo + 3, hi - 3, 24)
                    ax, bx = curve_x(a, rows), curve_x(b, rows)
                    if np.max(abs(ax - bx)) > 6: continue
                    ta = np.arctan((curve_x(a, rows+3) - curve_x(a, rows-3))/6)
                    tb = np.arctan((curve_x(b, rows+3) - curve_x(b, rows-3))/6)
                    if np.percentile(abs(ta-tb), 90) > np.deg2rad(20): continue
                    # Distinct parallel stripes with a dark gap are not duplicates.
                    bridge = ax[:, None] + (bx-ax)[:, None]*np.linspace(0, 1, 7)
                    x = np.rint(bridge).astype(int)
                    y = np.broadcast_to(np.rint(rows).astype(int)[:, None], x.shape)
                    inside = (x >= 0) & (x < mask.shape[1]) & (y >= 0) & (y < mask.shape[0])
                    if not inside.all() or np.mean(mask[y, x] > 0) < .9: continue
                    links.append((i, j))
            # Do not resolve competing continuations by iteration order.
            unique = [(i, j) for i, j in links if
                      sum(i in link for link in links) == 1 and
                      sum(j in link for link in links) == 1]
            if not unique: break
            i, j = unique[0]; a, b = candidates[i], candidates[j]
            near = max((a, b), key=lambda c: c['y_max'])
            ys = np.unique(np.r_[np.linspace(a['y_min'], a['y_max'], 48),
                                 np.linspace(b['y_min'], b['y_max'], 48)])
            xs = []
            for y in ys:
                owner = near if near['y_min'] <= y <= near['y_max'] else (b if near is a else a)
                xs.append(float(curve_x(owner, y)))
            joined = dict(near, knots=np.column_stack((xs, ys)).tolist(),
                          y_min=float(ys[0]), y_max=float(ys[-1]),
                          support=max(a['support'], b['support']),
                          score=min(a['score'], b['score']),
                          merged_fragments=a.get('merged_fragments', 1)+b.get('merged_fragments', 1))
            candidates = [c for k, c in enumerate(candidates) if k not in (i, j)] + [joined]
        return candidates

    def join_fragments(self, candidates, mask=None):
        # Connect pieces of the SAME stripe at adjacent endpoints, not across
        # parallel lanes. Competing continuations are deliberately left apart.
        candidates=list(candidates)
        while True:
            links=[]
            for i,a in enumerate(candidates):
                for j,b in enumerate(candidates):
                    if i==j or not a['y_max']<b['y_max']:continue
                    # A sparse far fragment must not poison a fully observed
                    # near boundary. Keep weak fragments separate for inspection.
                    if min(a['score'],b['score'])<.7:continue
                    gap=b['y_min']-a['y_max']
                    if not -44<=gap<=12:continue
                    y=(a['y_max']+b['y_min'])/2
                    error=abs(float(curve_x(a,y)-curve_x(b,y)))
                    sa=float((curve_x(a,a['y_max'])-curve_x(a,a['y_max']-6))/6)
                    sb=float((curve_x(b,b['y_min']+6)-curve_x(b,b['y_min']))/6)
                    overlap_lo,overlap_hi=max(a['y_min'],b['y_min']),min(a['y_max'],b['y_max'])
                    overlap_error=0.
                    if overlap_hi-overlap_lo>=12:
                        rows=np.linspace(overlap_lo,overlap_hi,8)
                        overlap_error=float(np.percentile(abs(curve_x(a,rows)-curve_x(b,rows)),90))
                    # Tangent angle is meaningful even for steep image curves.
                    angle=abs(float(np.arctan(sa)-np.arctan(sb)))
                    connected_bend=False
                    if mask is not None and 0<=gap<=12 and error<8 and angle<np.deg2rad(60):
                        # A tight observed bend can rotate its tangent across
                        # the scan-axis transition. Require actual paint along
                        # the short junction, not just compatible extrapolation.
                        start=np.array([curve_x(a,a['y_max']),a['y_max']])
                        end=np.array([curve_x(b,b['y_min']),b['y_min']])
                        sample=np.rint(np.linspace(start,end,max(3,int(np.linalg.norm(end-start))+1))).astype(int)
                        visible=[]
                        for x,y in sample:
                            visible.append(bool(mask[max(0,y-1):y+2,max(0,x-1):x+2].any()))
                        connected_bend=np.linalg.norm(end-start)<=15 and all(visible)
                    if error<12 and overlap_error<12 and (angle<np.deg2rad(35) or connected_bend):links.append((i,j))
            # A chain's middle fragment has one predecessor AND one successor;
            # that is not a fork. Require unique directed continuation.
            unique=[(i,j) for i,j in links
                    if sum(a==i for a,b in links)==1 and sum(b==j for a,b in links)==1]
            if not unique:break
            i,j=unique[0];a,b=candidates[i],candidates[j]
            ys=np.unique(np.r_[np.linspace(a['y_min'],a['y_max'],32),np.linspace(b['y_min'],b['y_max'],32)])
            xs=[]
            for y in ys:
                present=[c for c in (a,b) if c['y_min']<=y<=c['y_max']]
                # Preserve the current near fragment; averaging crossing tails
                # can fabricate paint between two distinct observations.
                owner=b if b in present else a
                xs.append(float(curve_x(owner,y)))
            combined=dict(a,knots=np.column_stack((xs,ys)).tolist(),y_min=float(ys[0]),y_max=float(ys[-1]),
                          support=a['support']+b['support'],score=min(a['score'],b['score']),
                          residual_px=max(a['residual_px'],b['residual_px']))
            candidates=[c for k,c in enumerate(candidates) if k not in (i,j)]+[combined]
        return candidates

    def fragment_identity_handoff(self, candidates, stamp):
        """Transfer identity only after co-observed, unique fragment evidence.

        Extrapolated tangents are used for association scoring only. Candidate
        pixels/points are never joined or extended by this function.
        """
        if not self.cfg.get('fragment_identity_handoff', False) or stamp is None:
            return
        by={c['id']:c for c in candidates}
        for identity in self.roles:
            previous=self.fragment_links.get(identity)
            if identity not in by:
                if previous and previous['count']>=3 and 0<(stamp-previous['stamp'])/1e9<=.25 and previous['successor'] in by:
                    b=by[previous['successor']]
                    # The successor must still match its last ACTUAL fragment.
                    old=previous['fragment'];lo=max(old['y_min'],b['y_min']);hi=min(old['y_max'],b['y_max'])
                    if hi-lo>=24 and b['score']>=.7:
                        y=np.linspace(lo,hi,12)
                        if np.percentile(abs(curve_x(old,y)-curve_x(b,y)),90)<12:
                            b['fragment_handoff_from_id']=b['id']
                            b['fragment_reference_points']=np.c_[curve_x(old,y),y].tolist()
                            b['fragment_reference_stamp_ns']=previous['stamp']
                            b['id']=identity
                            self.memory.pop(previous['successor'],None)
                continue
            a=by[identity];choices=[];bounded_grace=False
            if a['score']<.7:continue
            # Full observed end sections; no extrapolated lane points emitted.
            ay=np.linspace(a['y_min'],min(a['y_min']+30,a['y_max']),12)
            if np.ptp(ay)<24:continue
            af=np.polyfit(ay,curve_x(a,ay),1)
            for b in candidates:
                if b['id']==identity or b['id'] in self.roles or b['score']<.7:continue
                gap=a['y_min']-b['y_max']
                if gap<=0:continue
                grace=(gap>100 and previous and previous['count']>=3 and
                       previous['successor']==b['id'] and 0<(stamp-previous['stamp'])/1e9<=.15)
                if gap>100 and not grace:continue
                bys=np.linspace(max(b['y_min'],b['y_max']-30),b['y_max'],12)
                if np.ptp(bys)<24:continue
                bf=np.polyfit(bys,curve_x(b,bys),1);mid=(a['y_min']+b['y_max'])/2
                angle=abs(np.degrees(np.arctan(af[0])-np.arctan(bf[0])))
                distance=abs(np.polyval(af,mid)-np.polyval(bf,mid))/np.sqrt(1+af[0]**2)
                if angle<=10 and distance<=20:
                    if grace:bounded_grace=True
                    else:choices.append(b)
            if len(choices)!=1:
                # The shrinking near fragment may move just outside the
                # acquisition gap. Keep an ALREADY verified link for its
                # original 0.25s lifetime; never refresh it with weaker data.
                if not choices and bounded_grace:continue
                self.fragment_links.pop(identity,None);continue
            b=choices[0];same=previous and previous['successor']==b['id'] and 0<(stamp-previous['stamp'])/1e9<=.15
            self.fragment_links[identity]=dict(successor=b['id'],fragment=dict(b),stamp=stamp,count=previous['count']+1 if same else 1)
        self.fragment_links={i:h for i,h in self.fragment_links.items() if (stamp-h['stamp'])/1e9<=.25}

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
                if hi-lo<30:
                    # A short LK-predicted overlap must not suppress a valid
                    # overlap with the ACTUAL previous observation. Paint's
                    # aperture ambiguity can slide the prediction along it.
                    observed=pool[identity][0]
                    dlo,dhi=max(c['y_min'],observed['y_min']),min(c['y_max'],observed['y_max'])
                    if self.cfg.get('association_observed_overlap',False) and dhi-dlo>=30:
                        rows=np.linspace(dlo,dhi,10)
                        error=float(np.median(abs(curve_x(c,rows)-curve_x(observed,rows))))
                        if error<self.cfg['association_distance']:
                            overlap=(dhi-dlo)/max(c['y_max']-c['y_min'],observed['y_max']-observed['y_min'])
                            matches.append((0 if identity in self.roles else 1,error+20*(1-overlap),i,identity))
                    continue
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
        if self.cfg.get('fragment_identity_handoff',False):
            for i,c in enumerate(candidates):
                identity=c.get('mask_anchor_id')
                if identity in self.roles and identity in pool and stamp is not None and 0<(stamp-pool[identity][1])/1e9<=.15:
                    matches.append((-2,0.,i,identity))
        history_evidence = {}
        if self.cfg.get('association_observed_overlap', False) and stamp is not None:
            # A clipped latest fragment may hide overlap that was observed
            # one or two frames earlier. Keep at most five ACTUAL observations
            # within 0.25 s; never reuse their points as a current boundary.
            for identity in self.previous_pair or ():
                if any(m[3] == identity for m in matches):
                    continue
                choices = []
                for i, c in enumerate(candidates):
                    if c['score'] < .7:
                        continue
                    evidence = []
                    for old, seen in self.observation_history.get(identity, []):
                        age = (stamp-seen)/1e9
                        if not 0 < age <= .25:
                            continue
                        lo, hi = max(c['y_min'],old['y_min']), min(c['y_max'],old['y_max'])
                        if hi-lo < 30:
                            continue
                        rows = np.linspace(lo,hi,10)
                        error = float(np.median(abs(curve_x(c,rows)-curve_x(old,rows))))
                        if error < self.cfg['association_distance']:
                            overlap = (hi-lo)/max(c['y_max']-c['y_min'],old['y_max']-old['y_min'])
                            evidence.append((error+20*(1-overlap),age))
                    if evidence:
                        score,age = min(evidence)
                        choices.append((score,i,age))
                choices.sort()
                if choices and (len(choices)==1 or choices[1][0]-choices[0][0]>3.):
                    score,i,age = choices[0]
                    matches.append((0,score,i,identity))
                    history_evidence[i,identity] = age
        # Protect the last confirmed boundary's CURRENT near observation.
        # A longer far branch must not take its identity merely by overlap.
        for identity in self.previous_pair or ():
            if identity not in pool:continue
            old=pool[identity][0];hi=old['y_max'];lo=max(old['y_min'],hi-36.)
            if hi-lo<24:continue
            near=[]
            for i,c in enumerate(candidates):
                if c['y_max']<hi-6 or c['y_min']>lo:continue
                rows=np.linspace(lo,min(hi,c['y_max']),8)
                residual=float(np.percentile(abs(curve_x(c,rows)-curve_x(old,rows)),90))
                if residual<min(12.,self.cfg['association_distance']):near.append((residual,i))
            near.sort()
            if near and (len(near)==1 or near[1][0]-near[0][0]>3.):
                error,i=near[0]
                matches.append((-1,error,i,identity))
        used=set()
        for _,_,i,identity in sorted(matches):
            if 'id' not in candidates[i] and identity not in used:
                candidates[i]['id']=identity; used.add(identity)
                if (i,identity) in history_evidence:
                    candidates[i]['association_history_age_s']=history_evidence[i,identity]
        for c in candidates:
            if 'id' not in c: c['id']=self.next_id; self.next_id+=1
        self.fragment_identity_handoff(candidates,stamp)
        if stamp is not None:
            if self.cfg.get('association_observed_overlap',False):
                self.observation_history = {identity:[(old,seen) for old,seen in history
                    if (stamp-seen)/1e9 <= .25] for identity,history in self.observation_history.items()}
                self.observation_history = {i:h for i,h in self.observation_history.items() if h}
                for c in candidates:
                    self.observation_history[c['id']] = (self.observation_history.get(c['id'],[])+[(dict(c),stamp)])[-5:]
            retired={c['fragment_handoff_from_id'] for c in candidates if 'fragment_handoff_from_id' in c}
            self.memory={identity:(p,seen) for identity,p,seen in predictions if identity not in retired}
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
        if self.cfg.get('defer_far_pair_crossing', False) and self.previous_pair:
            known = list(self.previous_pair)
            far_rejection = any(p['ids'] == known and p['reason'] == 'CROSSING_OR_THIN_PAIR'
                                for p in rejected)
            by_id = {c['id']: c for c in candidates}
            if far_rejection and all(i in by_id for i in known):
                left, right = (by_id[i] for i in known)
                hi = min(left['y_max'], right['y_max'])
                lo = max(left['y_min'], right['y_min'], hi-self.cfg['common_span'])
                rows = np.linspace(lo, hi, 16)
                widths = curve_x(right, rows)-curve_x(left, rows)
                if (hi-lo >= self.cfg['common_span'] and hi >= height*self.cfg['near_row_min']
                        and min(left['score'], right['score']) >= .7
                        and np.all(widths >= self.cfg['width_min'])
                        and np.all(widths <= width*self.cfg['width_max_fraction'])):
                    # Do not accept this as a dual detection or update width.
                    # The existing metric near-corridor check must confirm all
                    # current alternatives through the actual pursuit target.
                    pairs.append(dict(ids=known, deferred_far_crossing=True))
                    return None, 'AMBIGUOUS_CORRIDOR', pairs, rejected
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
