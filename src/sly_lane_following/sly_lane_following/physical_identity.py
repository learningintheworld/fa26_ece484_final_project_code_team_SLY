"""Opt-in physical stripe association, separate from driving-lane roles.

Inputs are CURRENT undistorted RGB and unchanged detector candidates. Connected
polylines may be non-monotonic in image y; never feed them into x(y) selection.
No predicted pixels, path extension, command publishing or global odometry.
"""
import copy
import math
import cv2
import numpy as np
from .handoff import relative_to_path


def _unit(v):
    n=np.linalg.norm(v)
    return v/n if n>1e-6 else np.zeros(2)


def paint_bridge(gray, a, b, max_gap=80.):
    """Connect endpoints only along a narrow, fully observed bright ridge."""
    a,b=np.asarray(a,float),np.asarray(b,float)
    delta=b[0]-a[-1];gap=float(np.linalg.norm(delta))
    if not 4<=gap<=max_gap:return None
    direction=_unit(delta)
    ta=_unit(a[-1]-a[max(0,len(a)-6)])
    tb=_unit(b[min(5,len(b)-1)]-b[0])
    limit=math.cos(math.radians(30))
    if min(ta@direction,tb@direction,ta@tb)<limit:return None
    normal=np.array([-direction[1],direction[0]])
    line=np.linspace(a[-1],b[0],int(math.ceil(gap))+1)
    h,w=gray.shape
    if np.any(line[:,0]<10) or np.any(line[:,0]>w-11) or np.any(line[:,1]<10) or np.any(line[:,1]>h-11):return None
    def sample(points):
        return cv2.remap(gray,points[:,0].astype('float32')[None],points[:,1].astype('float32')[None],cv2.INTER_LINEAR)[0].astype(float)
    offsets=np.arange(-3.,4.)
    values=np.stack([sample(line+k*normal) for k in offsets])
    background=np.maximum(sample(line+9*normal),sample(line-9*normal))
    peak=values.max(0);contrast=peak-background
    supported=(contrast>=15)&(peak>=90)
    # Do not fill actual missing paint. Even a short unsupported interior
    # interval makes this a gap, not a same-frame observed connection.
    if not supported[2:-2].all():return None
    weights=np.maximum(values-background[None]-10,0)
    offset=(weights*offsets[:,None]).sum(0)/np.maximum(weights.sum(0),1)
    ridge=line+offset[:,None]*normal
    if np.max(np.linalg.norm(np.diff(ridge,axis=0),axis=1))>3:return None
    return dict(points=ridge.tolist(),gap_px=gap,contrast_min=float(contrast[2:-2].min()),
                source='current_raw_white_cross_sections')


def connect_observed(gray, candidates, confirmed_roles=None):
    """Separate physical polylines; input candidate IDs/points are immutable."""
    roles=confirmed_roles or {};groups=[dict(source_ids=[c['id']],points=copy.deepcopy(c['points']),joins=[]) for c in candidates]
    while True:
        proposals=[]
        for i,a in enumerate(groups):
            for j in range(i+1,len(groups)):
                b=groups[j]
                known={roles[k] for k in a['source_ids']+b['source_ids'] if k in roles}
                if len(known)>1:continue
                for ea in [0,1]:
                    aa=np.array(a['points']);aa=aa[::-1] if ea==0 else aa
                    for eb in [0,1]:
                        bb=np.array(b['points']);bb=bb[::-1] if eb==1 else bb
                        bridge=paint_bridge(gray,aa,bb)
                        if bridge is not None:proposals.append((i,j,ea,eb,aa,bb,bridge))
        # If an endpoint has several continuations, defer the branch entirely.
        count={}
        for i,j,ea,eb,*_ in proposals:
            count[i,ea]=count.get((i,ea),0)+1;count[j,eb]=count.get((j,eb),0)+1
        unique=[p for p in proposals if count[p[0],p[2]]==count[p[1],p[3]]==1]
        if not unique:break
        i,j,ea,eb,aa,bb,bridge=min(unique,key=lambda p:p[-1]['gap_px'])
        a,b=groups[i],groups[j]
        joined=dict(source_ids=a['source_ids']+b['source_ids'],
                    points=np.vstack([aa,np.array(bridge['points'])[1:-1],bb]).tolist(),
                    joins=a['joins']+b['joins']+[dict(from_ids=a['source_ids'],to_ids=b['source_ids'],**bridge)])
        groups=[g for k,g in enumerate(groups) if k not in (i,j)]+[joined]
    return groups


def estimate_motion(previous, current, projector, camera, dt):
    """Independent adjacent-frame ground SE(2); never chain motion transforms."""
    if not .005<dt<=.25 or previous.shape!=current.shape:return dict(valid=False,reason='MOTION_TIME_OR_SHAPE')
    h,w=previous.shape;mask=np.zeros_like(previous);mask[int(.646*h):int(.969*h),int(.039*w):int(.961*w)]=255
    p=cv2.goodFeaturesToTrack(previous,500,.01,8,mask=mask)
    if p is None:return dict(valid=False,reason='NO_GROUND_TEXTURE')
    z,ok,_=cv2.calcOpticalFlowPyrLK(previous,current,p,None,winSize=(31,31),maxLevel=3)
    if z is None:return dict(valid=False,reason='FLOW_FAILED')
    back,rev,_=cv2.calcOpticalFlowPyrLK(current,previous,z,None,winSize=(31,31),maxLevel=3)
    if back is None:return dict(valid=False,reason='FLOW_FAILED')
    u,v=p[:,0],z[:,0]
    keep=(ok[:,0]>0)&(rev[:,0]>0)&(np.linalg.norm(back[:,0]-u,axis=1)<.5)
    keep&=(v[:,0]>10)&(v[:,0]<w-10)&(v[:,1]>.635*h)&(v[:,1]<.99*h)
    A,va=projector.project(u,camera);B,vb=projector.project(v,camera)
    keep&=va&vb&(np.linalg.norm(A,axis=1)<15)&(np.linalg.norm(B,axis=1)<15)
    A,B=A[keep],B[keep]
    if len(A)<12:return dict(valid=False,reason='TOO_FEW_GROUND_TRACKS',matches=len(A))
    cv2.setRNGSeed(17)
    model,inliers=cv2.estimateAffinePartial2D(A,B,method=cv2.RANSAC,ransacReprojThreshold=.06,maxIters=3000,confidence=.999)
    if model is None:return dict(valid=False,reason='NO_MOTION_CONSENSUS')
    take=inliers[:,0].astype(bool);X,Y=A[take],B[take];mx,my=X.mean(0),Y.mean(0)
    U,_,Vt=np.linalg.svd((X-mx).T@(Y-my));rot=U@Vt
    if np.linalg.det(rot)<0:U[:,-1]*=-1;rot=U@Vt
    trans=my-mx@rot;errors=np.linalg.norm(X@rot+trans-Y,axis=1)
    scale=float(np.linalg.norm(model[0,:2]));p95=float(np.percentile(errors,95))
    valid=len(X)>=12 and take.mean()>=.5 and p95<=.12 and .97<=scale<=1.03
    valid=valid and np.ptp(X[:,0])>=1 and np.ptp(X[:,1])>=1
    return dict(valid=bool(valid),reason='RGB_MOTION' if valid else 'MOTION_UNCERTAIN',
                rotation=rot.tolist(),translation=trans.tolist(),inliers=len(X),matches=len(A),
                p95_m=p95,scale=scale,dt_s=dt,
                yaw_deg=float(-np.degrees(np.arctan2(rot[0,1],rot[0,0]))))


def _ground(points, projector, camera):
    q,good=projector.project(points,camera);good&=np.linalg.norm(q,axis=1)<15
    # No connector across invalid projection holes.
    ids=np.flatnonzero(good)
    if not len(ids):return np.empty((0,2))
    runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1);idx=max(runs,key=len)
    return q[idx]


def _resample(points):
    p=np.asarray(points,float)
    if len(p)<2:return p,np.zeros(len(p))
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
    keep=np.r_[True,np.diff(arc)>1e-5];p,arc=p[keep],arc[keep]
    sample=np.linspace(0,arc[-1],max(4,int(arc[-1]/.08)+1))
    return np.c_[np.interp(sample,arc,p[:,0]),np.interp(sample,arc,p[:,1])],sample


def association_score(old,current):
    if min(len(old),len(current))<4:return None
    old,oa=_resample(old);current,ca=_resample(current)
    good,d=relative_to_path(current,old);reverse,rd=relative_to_path(old,current)
    good&=abs(d)<=.15;reverse&=abs(rd)<=.15
    if good.sum()<4 or reverse.sum()<4:return None
    # Arc-uniform samples prevent a dense raw-paint connector outweighing
    # the original fragments. Partial-to-full matching supports observed merges.
    if good.mean()<.5 and reverse.mean()<.7:return None
    near=current[good]
    if np.linalg.norm(np.diff(near,axis=0),axis=1).sum()<.3:return None
    nearest=np.argmin(np.linalg.norm(near[:,None]-old[None,:],axis=2),axis=1)
    tangent=np.gradient(old,axis=0)[nearest];now=np.gradient(near,axis=0)
    cosine=np.abs(np.sum(tangent*now,axis=1))/np.maximum(np.linalg.norm(tangent,axis=1)*np.linalg.norm(now,axis=1),1e-6)
    if np.median(cosine)<math.cos(math.radians(20)):return None
    error=float(max(np.percentile(abs(d[good]),90),np.percentile(abs(rd[reverse]),90)))
    return dict(error_m=error,coverage=float(good.mean()),previous_coverage=float(reverse.mean()),
                samples=int(good.sum()),old_interval=[float(oa[reverse].min()),float(oa[reverse].max())],
                new_interval=[float(ca[good].min()),float(ca[good].max())])


def _disjoint(a,b):
    return min(a[1],b[1])-max(a[0],b[0])<.15


class PhysicalStripeTracker:
    """Physical aliases do not authorize a lane-role change or actuator output."""
    def __init__(self,projector):
        self.projector=projector;self.previous=None;self.next_id=1

    def update(self,gray,candidates,camera,stamp,*,confirmed_roles=None,excluded_ids=(),verified_dual_ids=()):
        roles=confirmed_roles or {};groups=connect_observed(gray,candidates,roles)
        previous=self.previous;motion=dict(valid=False,reason='FIRST_OBSERVATION');matches=[]
        if previous:
            dt=(stamp-previous['stamp'])/1e9
            motion=estimate_motion(previous['gray'],gray,self.projector,camera,dt)
            if motion['valid']:
                for i,old in enumerate(previous['groups']):
                    q=_ground(old['points'],self.projector,camera)
                    warped=q@np.array(motion['rotation'])+motion['translation']
                    for j,new in enumerate(groups):
                        score=association_score(warped,_ground(new['points'],self.projector,camera))
                        if score:matches.append(dict(previous=i,current=j,**score))
        accepted={}
        for j,g in enumerate(groups):
            incoming=sorted([m for m in matches if m['current']==j],key=lambda m:m['error_m'])
            if not incoming:continue
            best=incoming[0]
            close=[m for m in incoming if m['error_m']-best['error_m']<.05]
            pids={previous['groups'][m['previous']]['physical_id'] for m in close}
            if len(pids)>1:
                # Several old fragments may merge only with a CURRENT paint
                # bridge and disjoint supporting intervals, never by proximity.
                if not g['joins'] or any(not _disjoint(a['new_interval'],b['new_interval'])
                     for i,a in enumerate(close) for b in close[i+1:]):continue
            outgoing=[m for m in matches if m['previous']==best['previous'] and m['current']!=j and m['error_m']-best['error_m']<.05]
            if outgoing:
                old=previous['groups'][best['previous']]
                # A previously observed connected stripe may appear as two
                # fragments again; keep its physical ID without filling the gap.
                if not old.get('connected_once') or any(not _disjoint(best['old_interval'],m['old_interval']) for m in outgoing):continue
            accepted[j]=close
        for j,g in enumerate(groups):
            links=accepted.get(j,[]);parents=[previous['groups'][m['previous']] for m in links]
            old=parents[0] if parents else None
            if parents:
                g.update(physical_id=min(p['physical_id'] for p in parents),motion_matches=links,
                         observations=max(p.get('observations',1) for p in parents)+1,
                         parent_physical_ids=sorted({p['physical_id'] for p in parents}))
            else:
                g.update(physical_id=self.next_id,observations=1,parent_physical_ids=[]);self.next_id+=1
            g['connected_once']=bool(g['joins']) or any(p.get('connected_once') for p in parents)
            known={roles[i] for i in g['source_ids'] if i in roles}
            inherited={p['role'] for p in parents if p.get('role')}
            blocked=bool(set(g['source_ids'])&set(excluded_ids))
            if len(known|inherited)>1:
                g.update(role=None,role_state='ROLE_CONFLICT')
            elif blocked or any(p.get('needs_role_reconfirmation') for p in parents):
                g.update(role=None,role_state='EXCLUDED_LINEAGE_REQUIRES_CURRENT_ROLE_EVIDENCE',needs_role_reconfirmation=True)
                # A fresh accepted DUAL geometry may re-establish a role,
                # unlike yaw direction, elapsed time or an arbitrary new ID.
                dual_ids=set(verified_dual_ids)
                other=[q for q in groups if q is not g and set(q['source_ids'])&dual_ids]
                current_lineage={p['physical_id'] for p in parents}
                other_lineage={previous['groups'][m['previous']]['physical_id']
                    for k,q in enumerate(groups) if q is not g and set(q['source_ids'])&dual_ids
                    for m in accepted.get(k,[])} if previous else set()
                verified=(not blocked and len(known)==1 and bool(set(g['source_ids'])&dual_ids)
                          and len(other)==1 and not(current_lineage&other_lineage)
                          and len({roles[i] for i in dual_ids if i in roles})==2)
                previous_votes=max((q.get('dual_role_votes',0) for q in parents),default=0)
                g['dual_role_votes']=previous_votes+1 if verified else 0
                if g['dual_role_votes']>=3:
                    g.update(role=next(iter(known)),role_state='RECONFIRMED_BY_CURRENT_DUAL_GEOMETRY',needs_role_reconfirmation=False)
            elif known:
                g.update(role=next(iter(known)),role_state='CURRENT_ACCEPTED_ROLE')
            elif inherited:
                g.update(role=next(iter(inherited)),role_state='MOTION_ASSOCIATED_ROLE')
            else:g.update(role=None,role_state='ROLE_UNCONFIRMED')
            g['excluded_source_ids']=sorted(set(g['source_ids'])&set(excluded_ids))
        self.previous=dict(stamp=int(stamp),gray=gray.copy(),groups=copy.deepcopy(groups))
        return dict(groups=groups,motion=motion,associations=matches,
                    scope='physical_observation_only_not_driving_authorization')

    def annotate(self,result,gray,camera,stamp,**kwargs):
        """Non-destructive opt-in metadata: existing selectors/controllers unchanged."""
        output=copy.deepcopy(result)
        output['physical_identity']=self.update(gray,result['candidates'],camera,stamp,**kwargs)
        return output
