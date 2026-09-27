"""Real-time Unreal environment. Privileged geometry + LiDAR + IMU, no RGB learning."""
import bisect,json,math,time,uuid
import numpy as np
from rl_demo import Track,ROOT,wrap
from rgb_imu_client import SensorClient
from vehicle_settings import apply_environment,validate_vehicle_settings
from server import lidar_points

class UnrealEnv:
    def __init__(self,vehicle,cfg):
        validate_vehicle_settings(vehicle)
        self.vehicle=vehicle;self.cfg=cfg;self.track=Track(ROOT/'track/generated/track.json')
        self.points=np.asarray(self.track.points);self.actions=[(0.,0.)]+[(vehicle['speed_mps']*f,s) for f in cfg['speed_fractions'] for s in cfg['steering_actions_rad']]
        self.client=SensorClient(vehicle['udp_port']);self.last=None;self.progress=0.;self.best_progress=0.;self.stuck_since=0.;self.contact_since=None
    def packet(self,command=None):
        return self.client.request(dict(command or {},sensors=True),lambda p:p.get('schema')=='f1_sensors_v1')
    def geometry(self,p):
        origin=self.track.data['ue_origin_cm'];x=p['x_m']+(10000-origin[0])/100;y=p['y_m']+origin[1]/100
        # Project only nearby segments; vectorized nearest vertex avoids 7201 Python iterations per step.
        nearest=int(np.argmin((self.points[:,0]-x)**2+(self.points[:,1]-y)**2)); best=None
        for i in [(nearest-1)%len(self.points),nearest]:
            a=self.points[i];b=self.points[(i+1)%len(self.points)];v=b[:2]-a[:2];l2=float(v@v)
            t=max(0,min(1,float((np.array([x,y])-a[:2])@v)/max(l2,1e-12)))
            foot=a[:2]+t*v;dist=float(np.sum((foot-[x,y])**2))
            h=math.atan2(v[1],v[0]);cross=-(x-foot[0])*math.sin(h)+(y-foot[1])*math.cos(h)
            station=(self.track.station[i]+t*math.dist(a,b))%self.track.length
            if best is None or dist<best[0]: best=(dist,h,cross,station)
        _,h,cross,station=best;err=wrap(p['yaw_rad']-h)
        envelope=.30*abs(math.sin(err))+.125*abs(math.cos(err));half=self.track.data['width_m']/2
        return dict(station=station,cross=cross,heading_error=err,left_margin=half-cross-envelope,right_margin=half+cross-envelope,x=x,y=y)
    def observe(self,p):
        g=self.geometry(p);imu=p['imu'];gyro=imu['angular_velocity_radps'];acc=imu['linear_acceleration_mps2']
        speed_scale=max(.5,self.vehicle['speed_mps'])
        obs=[g['cross']/.6,math.sin(g['heading_error']),math.cos(g['heading_error']),p['forward_speed_mps']/speed_scale,
            gyro['z']/5,acc['x']/20,acc['y']/20,acc['z']/20,g['left_margin']/.6,g['right_margin']/.6]
        yaw=p['yaw_rad'];c=math.cos(yaw);s=math.sin(yaw)
        for distance in [1,2,4,8]:
            i=min(len(self.points)-1,bisect.bisect_left(self.track.station,(g['station']+distance)%self.track.length))
            dx,dy=self.points[i,0]-g['x'],self.points[i,1]-g['y']
            obs.extend([(c*dx+s*dy)/8,(-s*dx+c*dy)/8,(self.points[i,2]-p['z_m'])/2])
        obs.extend(float(w['in_contact']) for w in p['wheels'])
        for key in ['front_scan','rear_scan']:
            scan=p[key];ranges=scan['ranges_m'];cap=scan['range_max_m']
            # 12 sectors per LiDAR: nearest valid range plus observed-return mask.
            for chunk in np.array_split(np.asarray(ranges,dtype=object),12):
                valid=[float(v) for v in chunk if v is not None and math.isfinite(v) and scan['range_min_m']<=v<=cap]
                obs.extend([min(valid,default=cap)/cap,float(bool(valid))])
        result=np.clip(np.asarray(obs,dtype=np.float32),-5,5)
        if not np.isfinite(result).all(): raise RuntimeError('Non-finite observation')
        return result,g
    def validate_packet(self,p):
        if not p.get('physics_valid'): raise RuntimeError('Unreal physics unavailable')
        for key in ['imu','front_scan','rear_scan']:
            sensor=p.get(key,{})
            if not sensor.get('valid') or not 0<=p['time_s']-sensor.get('stamp_s',-100)<=.5:
                raise RuntimeError(f'Stale or invalid {key}')
    def reset(self):
        token=uuid.uuid4().hex
        probe=self.packet()
        if probe.get('episode_api_version',0)<1:
            raise RuntimeError('Unreal reset API missing. Install the DQN plugin build and restart Unreal.')
        # A previous brake reply may still be in the socket. Match the reset token,
        # never merely the common telemetry schema. Same-token retries are idempotent.
        p=self.client.request(dict(reset_episode=token,speed_mps=0,steering_rad=0,sensors=True),
            lambda p:p.get('schema')=='f1_sensors_v1' and p.get('reset_token')==token)
        start=time.monotonic(); sim_start=p['time_s'];good=0
        while time.monotonic()-start<8:
            time.sleep(.05);p=self.packet(dict(speed_mps=0,steering_rad=0))
            if p['time_s']-sim_start<.75: continue
            try: self.validate_packet(p)
            except RuntimeError: continue
            q=p['imu']['orientation_ground_truth_xyzw']
            settled=all(w['in_contact'] for w in p['wheels']) and abs(p['forward_speed_mps'])<.1 and 1-2*(q['x']**2+q['y']**2)>.95
            good=good+1 if settled else 0
            if good>=5: break
        else: raise RuntimeError('Reset did not settle upright with four wheel contacts within 8 seconds')
        p=apply_environment(self.client,self.vehicle);self.validate_packet(p)
        obs,g=self.observe(p);self.last=(p,g);self.progress=0.;self.best_progress=0.;self.stuck_since=p['time_s'];self.contact_since=None
        return obs,p,g
    def step(self,index):
        speed,steer=self.actions[index];old,og=self.last;started=time.monotonic()
        while True:
            p=self.packet(dict(speed_mps=speed,steering_rad=steer))
            if p['time_s']-old['time_s']>=self.cfg['decision_seconds']: break
            if time.monotonic()-started>2: raise RuntimeError('Simulation paused or stalled')
            time.sleep(.025)
        self.validate_packet(p);dt=p['time_s']-old['time_s']
        if dt>1: raise RuntimeError('Simulation transition gap exceeded 1 second')
        obs,g=self.observe(p);length=self.track.length
        ds=(g['station']-og['station']+length/2)%length-length/2
        if abs(ds)>max(1.,self.vehicle['max_speed_mps']*dt*1.5): raise RuntimeError('Discontinuous track projection')
        self.progress+=ds
        if self.progress>self.best_progress+.1: self.best_progress=self.progress;self.stuck_since=p['time_s']
        q=p['imu']['orientation_ground_truth_xyzw'];reason=None
        if min(g['left_margin'],g['right_margin'])<0: reason='off_track'
        elif 1-2*(q['x']**2+q['y']**2)<.8: reason='rollover'
        elif any(.30<x<.60 and abs(y)<.20 for x,y,_ in lidar_points(p)): reason='obstacle_proximity'
        if all(w['in_contact'] for w in p['wheels']): self.contact_since=None
        elif self.contact_since is None: self.contact_since=p['time_s']
        elif p['time_s']-self.contact_since>.5: reason=reason or 'airborne'
        if p['time_s']-self.stuck_since>8: reason=reason or 'no_progress'
        # A lap requires a full net distance and returning to the start vicinity.
        if not reason and self.progress>=length-.1: reason='lap_complete'
        margin=min(g['left_margin'],g['right_margin'])
        reward=ds-.05*dt-.5*max(0,.08-margin)*dt-.005*abs(steer)*dt
        if reason: reward+=20 if reason=='lap_complete' else -5
        self.last=(p,g)
        return obs,reward,bool(reason),dict(packet=p,geometry=g,dt=dt,reason=reason,progress_m=self.progress,speed_command=speed,steering_command=steer)
    def stop(self):
        # Consume the brake reply so it cannot masquerade as the next observation.
        try:self.packet(dict(speed_mps=0,steering_rad=0))
        except (OSError,TimeoutError):self.client.sock.send(b'{"speed_mps":0,"steering_rad":0}')
    def close(self):
        try:self.stop()
        finally:self.client.close()
    def show_line(self,rows):
        self.packet(dict(clear_racing_line=True,speed_mps=0,steering_rad=0))
        points=[];previous=None
        for row in rows:
            p=row['packet'];point=[p['x_m'],p['y_m'],p['z_m']+.015]
            if previous is None or math.dist(point,previous)>.1: points.append(point);previous=point
        if len(points)>4000: points=points[::math.ceil(len(points)/4000)]
        for i in range(0,len(points),20):self.packet(dict(racing_line_points=points[i:i+20],speed_mps=0,steering_rad=0))
