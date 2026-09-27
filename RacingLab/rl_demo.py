"""Real Unreal transitions: tabular Q-learning of residual steering over pure pursuit.

No surrogate physics, pose teleportation or fabricated training data. Train is a
continuing task; safety termination brakes and requires Stop/Play to restart.
"""
import argparse, bisect, json, math, random, sys, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
sys.path.insert(0,str(ROOT/'simulation_ml/integrations/rapyuta'))
sys.path.insert(0,str(ROOT/'simulation_ml/integrations/rapyuta/dashboard'))
from rgb_imu_client import SensorClient
from server import lidar_points
from vehicle_settings import validate_vehicle_settings,apply_environment

def wrap(a): return (a+math.pi)%(2*math.pi)-math.pi
def clamp(x,a,b): return max(a,min(b,x))

class Track:
    def __init__(self,path):
        self.data=json.loads(Path(path).read_text()); self.points=self.data['centerline_m']
        self.station=[0.]
        for a,b in zip(self.points,self.points[1:]+self.points[:1]):
            self.station.append(self.station[-1]+math.dist(a,b))
        self.length=self.station[-1]
    def observe(self,p,lookahead):
        origin=self.data['ue_origin_cm']
        x=p['x_m']+(10000-origin[0])/100; y=p['y_m']+origin[1]/100
        i=min(range(len(self.points)),key=lambda i:(self.points[i][0]-x)**2+(self.points[i][1]-y)**2)
        a=self.points[i]; b=self.points[(i+1)%len(self.points)]
        heading=math.atan2(b[1]-a[1],b[0]-a[0]); error=wrap(p['yaw_rad']-heading)
        cross=-(x-a[0])*math.sin(heading)+(y-a[1])*math.cos(heading)
        j=min(len(self.points)-1,bisect.bisect_left(self.station,(self.station[i]+lookahead)%self.length))
        target=self.points[j]; dx=target[0]-x; dy=target[1]-y
        alpha=wrap(math.atan2(dy,dx)-p['yaw_rad'])
        steer=math.atan2(2*.37*math.sin(alpha),max(.1,math.hypot(dx,dy)))
        margin=self.data['width_m']/2-abs(cross)-(.30*abs(math.sin(error))+.125*abs(math.cos(error)))
        return dict(station=self.station[i],cross=cross,heading_error=error,baseline=steer,margin=margin)

def state(o):
    return f"{bisect.bisect_left([-.15,-.05,.05,.15],o['cross'])},{bisect.bisect_left([-.20,-.06,.06,.20],o['heading_error'])}"

def reward(old,new,length,dt,action):
    progress=(new['station']-old['station']+length/2)%length-length/2
    return (progress/max(dt,.001)-2*abs(new['cross'])-.5*abs(new['heading_error'])-.2*abs(action))*dt

def terminal(p,o):
    if not p.get('physics_valid'): return 'invalid_physics'
    if o['margin']<.08: return 'track_margin'
    imu=p.get('imu',{})
    if not imu.get('valid') or p['time_s']-imu.get('stamp_s',-100)>.5: return 'invalid_imu'
    q=imu['orientation_ground_truth_xyzw']
    if 1-2*(q['x']**2+q['y']**2)<.85: return 'excessive_tilt'
    if not all(w['in_contact'] for w in p['wheels']): return 'lost_wheel_contact'
    scan=p.get('front_scan',{})
    if not scan.get('valid') or p['time_s']-scan.get('stamp_s',-100)>.5: return 'stale_lidar'
    if any(.30<x<.85 and abs(y)<.23 for x,y,_ in lidar_points(p)): return 'lidar_stop'
    return None

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['train','evaluate','baseline','diagnose','stop','configure'])
    parser.add_argument('--seconds',type=float); args=parser.parse_args()
    cfg=json.loads((HERE/'settings.json').read_text())
    if args.mode=='stop':
        # A bad experiment setting must never prevent an explicit stop command.
        stop_client=SensorClient(cfg.get('udp_port',7449))
        try: stop_client.sock.send(b'{"speed_mps":0,"steering_rad":0}')
        finally: stop_client.close()
        print('Stop sent. Also terminate any controller still sending drive commands.')
        return
    for key in ['speed_mps','lookahead_m','control_hz','alpha','gamma','epsilon']:
        if not math.isfinite(cfg[key]): raise ValueError(f'Nonfinite {key}')
    validate_vehicle_settings(cfg)
    if not (.3<=cfg['lookahead_m']<=2 and 5<=cfg['control_hz']<=30):
        raise ValueError('Demo limits: lookahead [.3,2] m, control [5,30] Hz')
    if not all(0<=cfg[k]<=1 for k in ['alpha','gamma','epsilon']): raise ValueError('Learning rates must be [0,1]')
    actions=cfg['residual_actions_rad']
    if not actions or 0 not in actions or any(not math.isfinite(a) or abs(a)>.08 for a in actions):
        raise ValueError('Actions must include zero and be within +/-0.08 rad')
    out=HERE/'runs';out.mkdir(exist_ok=True); policy_path=HERE/'policy.json'
    rng=random.Random(cfg['seed']); table={}; visits={}
    if args.mode in ('evaluate','train') and policy_path.exists():
        policy=json.loads(policy_path.read_text())
        if policy['actions']!=actions: raise ValueError('Actions changed: archive policy.json before retraining')
        table=policy['q']; visits=policy.get('visits',{})
    elif args.mode=='evaluate': raise RuntimeError('Train first: policy.json is missing')
    track=Track(ROOT/'track/generated/track.json'); client=SensorClient(cfg['udp_port'])
    stamp=time.strftime('%Y%m%d_%H%M%S'); rows=[]; reason='duration'; updates=0
    seconds=args.seconds if args.seconds is not None else cfg['train_seconds' if args.mode=='train' else 'evaluate_seconds']
    if not math.isfinite(seconds) or seconds<=0: raise ValueError('Duration must be positive and finite')
    prior=None; last_sim=None; total_reward=0.; start=time.monotonic(); last_print=start
    try:
        if args.mode=='stop': return
        # Establish current state while braking. Do not steer a car already outside limits.
        p=apply_environment(client,cfg)
        print(f"Vehicle configured: target {cfg['speed_mps']:.4f} m/s ({cfg['speed_mps']/.44704:.2f} mph), limit {p['max_speed_mps']:.4f} m/s, tire grip {p['front_tire_grip_multiplier']:.3f}",flush=True)
        if args.mode=='configure': return
        while time.monotonic()-start<seconds:
            tick=time.monotonic(); o=track.observe(p,cfg['lookahead_m']); s=state(o)
            reason_now=terminal(p,o)
            if last_sim is not None and p['time_s']<=last_sim: reason_now='paused_or_stale'
            if prior:
                po,ps,ai,pt=prior; dt=p['time_s']-pt
                r=reward(po,o,track.length,dt,actions[ai]) if dt>0 else -1
                if reason_now: r-=2
                total_reward+=r
                if args.mode=='train':
                    q=table.setdefault(ps,[0.]*len(actions)); future=0 if reason_now else max(table.get(s,[0.]*len(actions)))
                    q[ai]+=cfg['alpha']*(r+cfg['gamma']*future-q[ai]); updates+=1
                    visits[ps]=visits.get(ps,0)+1
            if reason_now: reason=reason_now; rows.append(dict(telemetry=p,observation=o,terminal=reason)); break
            ai=actions.index(0)
            if args.mode in ('train','evaluate'):
                if args.mode=='train' and rng.random()<cfg['epsilon']: ai=rng.randrange(len(actions))
                else:
                    q=table.get(s,[0.]*len(actions)); ai=max(range(len(actions)),key=lambda i:(q[i],-abs(actions[i])))
            steering=clamp(o['baseline']+(actions[ai] if args.mode in ('train','evaluate') else 0),-.462,.462)
            if args.mode=='diagnose': steering=0
            rows.append(dict(telemetry=p,observation=o,state=s,action=ai,steering_rad=steering))
            prior=(o,s,ai,p['time_s']);last_sim=p['time_s']
            time.sleep(max(0,1/cfg['control_hz']-(time.monotonic()-tick)))
            p=client.request(dict(speed_mps=cfg['speed_mps'],steering_rad=steering,sensors=True),lambda r:r.get('schema')=='f1_sensors_v1')
            if time.monotonic()-last_print>5:
                print(f"{args.mode}: {len(rows)} samples, cross {o['cross']:+.3f}m, margin {o['margin']:.3f}m, updates {updates}",flush=True);last_print=time.monotonic()
    except KeyboardInterrupt: reason='interrupted'
    except Exception as exc:
        reason=f'error: {exc}'; raise
    finally:
        try: client.sock.send(b'{"speed_mps":0,"steering_rad":0}')
        finally: client.close()
        if args.mode=='train':
            policy_path.write_text(json.dumps(dict(algorithm='tabular Q-learning residual steering',actions=actions,q=table,visits=visits,config=cfg),indent=2))
        observations=[r['observation'] for r in rows]
        summary=dict(mode=args.mode,termination=reason,samples=len(rows),updates=updates,total_reward=total_reward,
            elapsed_wall_s=time.monotonic()-start,states=len(table),settings=cfg,
            mean_abs_cross_m=sum(abs(o['cross']) for o in observations)/max(1,len(observations)),
            max_abs_cross_m=max([abs(o['cross']) for o in observations],default=0),
            minimum_margin_m=min([o['margin'] for o in observations],default=0),
            progress_m=sum((b['station']-a['station']+track.length/2)%track.length-track.length/2 for a,b in zip(observations,observations[1:])))
        (out/f'{stamp}_{args.mode}.json').write_text(json.dumps(dict(summary=summary,rows=rows)))
        (out/'latest_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
    if reason!='duration': sys.exit(2)
if __name__=='__main__': main()
