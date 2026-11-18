"""Opt-in right-boundary gap proposal; no pixels or width measurements invented.

Caller must supply the confirmed right identity. Returns a separate proposal,
never mutates detector input. Interpolation is bounded by two CURRENT observed
endpoints, with a unique continuation and three-frame agreement.
"""
import copy
import cv2
import numpy as np


class RightGapBridge:
    def __init__(self):
        self.previous=None
        self.vote=None
        self.count=0
        self.reason='UNINITIALIZED'

    def propose(self,candidate,mask,stamp,confirmed_right_id,projector,camera):
        self.reason='NO_UNIQUE_SUPPORTED_CONTINUATION'
        if candidate is None or candidate.get('id')!=confirmed_right_id or candidate.get('score',0)<.7:
            self.previous=None;self.vote=None;self.count=0
            self.reason='NO_CONFIRMED_RIGHT_OBSERVATION';return None
        p=np.asarray(candidate['points'],float)
        if len(p)<6 or np.ptp(p[:,1])<40 or np.any(np.diff(p[:,1])<=0):
            self.vote=None;self.count=0;return None
        previous=self.previous;self.previous=(stamp,candidate['id'],p.copy())
        continuous=False
        if previous and previous[1]==candidate['id'] and 0<(stamp-previous[0])/1e9<=.25:
            old=previous[2];lo=max(old[0,1],p[0,1]);hi=min(old[-1,1],p[-1,1])
            if hi-lo>=24:
                ys=np.linspace(max(lo,hi-60),hi,12)
                continuous=np.max(abs(np.interp(ys,old[:,1],old[:,0])-np.interp(ys,p[:,1],p[:,0])))<=16
        if not continuous:self.vote=None;self.count=0
        a=p[0];near=p[p[:,1]<=a[1]+24]
        if len(near)<3:near=p[:4]
        near_slope=float(np.polyfit(near[:,1],near[:,0],1)[0])
        n,labels,stats,_=cv2.connectedComponentsWithStats((mask>0).astype('uint8'),8)
        proposals=[]
        for i in range(1,n):
            if stats[i,cv2.CC_STAT_AREA]<20:continue
            yy,xx=np.nonzero(labels==i)
            # Entire fragment must lie farther ahead, not overlap the anchor.
            if yy.max()>=a[1]-3 or yy.max()<a[1]-.08*mask.shape[0]:continue
            ys=np.unique(yy);q=np.array([[np.median(xx[yy==y]),y] for y in ys],float)
            if len(q)<8 or np.ptp(q[:,1])<8:continue
            b=q[-1];gap=float(np.linalg.norm(a-b))
            if not 3<gap<=.09*mask.shape[0]:continue
            far=q[q[:,1]>=b[1]-20];far_slope=float(np.polyfit(far[:,1],far[:,0],1)[0])
            chord=(a[0]-b[0])/(a[1]-b[1])
            angles=np.arctan([near_slope,far_slope,chord])
            if np.ptp(angles)>np.deg2rad(20):continue
            if abs(b[0]+far_slope*(a[1]-b[1])-a[0])>12:continue
            if abs(a[0]-near_slope*(a[1]-b[1])-b[0])>12:continue
            ground,valid=projector.project([b,a],camera)
            if not valid.all():continue
            gap_m=float(np.linalg.norm(ground[1]-ground[0]))
            if gap_m>3.:continue
            t=np.linspace(0,1,max(3,int(gap)+1));dy=a[1]-b[1]
            xs=(2*t**3-3*t**2+1)*b[0]+(t**3-2*t**2+t)*dy*far_slope+(-2*t**3+3*t**2)*a[0]+(t**3-t**2)*dy*near_slope
            bridge=np.c_[xs,b[1]+t*dy]
            if np.any(abs(xs-(b[0]+t*(a[0]-b[0])))>6):continue
            proposals.append((q,bridge,gap,gap_m))
        if len(proposals)!=1:
            self.vote=None;self.count=0
            self.reason='AMBIGUOUS_GAP_CONTINUATION' if proposals else self.reason
            return None
        q,bridge,gap,gap_m=proposals[0];endpoint=q[-1]
        self.count=self.count+1 if continuous and self.vote is not None and np.linalg.norm(endpoint-self.vote)<16 else 1
        self.vote=endpoint
        if self.count<3:self.reason='CONFIRMING_GAP_CONTINUATION';return None
        out=copy.deepcopy(candidate)
        out['points']=np.vstack([q,bridge[1:-1],p]).tolist()
        out['observed_points']=np.vstack([q,p]).tolist()
        out['near_observed_points']=p.tolist()
        out['inferred_segments']=[bridge.tolist()]
        out['gap_pixels']=gap;out['gap_metres']=gap_m
        out['width_measurement_allowed']=False
        # Conservative geometric sensitivity bound, not calibrated confidence.
        out['bridge_error_bound_m']=gap_m*np.tan(np.deg2rad(20))/2
        out['bridge_kind']='bounded_current_right_gap'
        self.reason='BRIDGED_CURRENT_RIGHT_GAP'
        return out
