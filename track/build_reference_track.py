"""Reference-inspired circuit, not a survey reconstruction of the perspective image."""
from pathlib import Path
import json
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import brentq
from scipy.ndimage import gaussian_filter1d

OUT = Path(__file__).parent / 'generated'
OUT.mkdir(exist_ok=True)
# Hand-traced landmarks from the displayed reference; image coordinates, arbitrary units.
anchors = np.array([
    [1580,750],[1770,360],[1730,335],[1410,485],[1040,372],[930,370],
    [810,334],[622,330],[556,292],[611,272],[640,252],[649,228],
    [622,208],[550,211],[523,195],[535,160],[544,144],
    [500,110],[371,121],[211,104],[177,109],[163,132],[181,160],
    [219,191],[294,342],[352,519],
    [447,516],[699,439],[741,443],[742,468],[687,488],[561,511],
    [524,545],[539,559],[702,515],[884,481],[1064,494],[1151,535],
    [1144,581],[987,683],[810,806],[835,863],[1020,1014],[1198,1127],
    [1270,1123],[1368,1012]
], dtype=float)
closed = np.vstack([anchors, anchors[0]])
t = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
curve = CubicSpline(t, closed, bc_type='periodic')
raw = curve(np.linspace(0, t[-1], 1800, endpoint=False))
raw[:,1] *= -1
raw -= raw[0]
distance = np.r_[0,np.cumsum(np.linalg.norm(np.diff(raw, axis=0),axis=1))]
u = distance/(distance[-1]+np.linalg.norm(raw[-1]-raw[0]))
# Smooth invented profile: a crest at the first hairpin and a second high section.
# Total height span scales the image legend's 133 ft at 3.4 miles down to 360 m.
height_range = 133*.3048/(3.4*1609.344)*360
profile = .6*np.cos(2*np.pi*(u-.06)) + .4*np.cos(4*np.pi*(u-.06))
z = (profile-profile.min())/np.ptp(profile)*height_range + .15
delta = np.roll(raw,-1,axis=0)-raw
dz = np.roll(z,-1)-z
scale = brentq(lambda k: np.sqrt(np.sum((delta*k)**2,axis=1)+dz**2).sum()-360,.001,1)
xy = raw*scale
# Round only overly tight image-traced corners so the 1.2 m ribbon cannot fold
# over itself on its inside edge. Keep the broad reference shape unchanged.
for _ in range(200):
    d = np.roll(xy,-1,axis=0)-xy
    heading = np.arctan2(d[:,1],d[:,0])
    turn = np.angle(np.exp(1j*(heading-np.roll(heading,1))))
    radius = np.linalg.norm(d,axis=1)/np.maximum(abs(turn),1e-8)
    if radius.min() > .8:
        break
    weights = np.minimum(3*gaussian_filter1d((radius<.9).astype(float),6,mode='wrap'),1)
    xy += .4*weights[:,None]*(gaussian_filter1d(xy,3,axis=0,mode='wrap')-xy)
xy -= xy[0]
dxy = np.roll(xy,-1,axis=0)-xy
xy *= brentq(lambda k: np.sqrt(np.sum((dxy*k)**2,axis=1)+dz**2).sum()-360,.5,2)
xyz = np.c_[xy,z]
tangent = np.roll(xy,-1,axis=0)-np.roll(xy,1,axis=0)
tangent /= np.linalg.norm(tangent,axis=1)[:,None]
normal = np.c_[-tangent[:,1],tangent[:,0]]
width = 1.2
left = xyz.copy();left[:,:2] += normal*width/2
right = xyz.copy();right[:,:2] -= normal*width/2

def ribbon(name, a, b):
    # UE 5.8 OBJ import: place mesh at (15000,0,0), Roll=+90 degrees,
    # unit scale. This yields UE=(15000+100*x,-100*y,100*z).
    verts=np.empty((len(a)*2,3));verts[0::2]=a;verts[1::2]=b
    obj=verts[:,[0,2,1]]*100
    obj[:,2] *= -1
    lines=['o '+name]+[f'v {x:.6f} {y:.6f} {z:.6f}' for x,y,z in obj]
    for i in range(len(a)):
        j=(i+1)%len(a);a0=2*i+1;b0=2*i+2;a1=2*j+1;b1=2*j+2
        lines += [f'f {a0} {b0} {b1}',f'f {a0} {b1} {a1}']
    (OUT/(name+'.obj')).write_text('\n'.join(lines)+'\n')

ribbon('asphalt',left,right)
for label,edge in [('bank_left',left),('bank_right',right)]:
    top=edge.copy();top[:,2]-=.01
    bottom=edge.copy();bottom[:,2]=-.05
    ribbon(label,top,bottom)
for label,edge,sign in [('white_left',left,-1),('white_right',right,1)]:
    inner=edge.copy();inner[:,:2]+=normal*sign*.06
    edge=edge.copy();edge[:,2]+=.0015;inner[:,2]+=.0015
    ribbon(label,edge,inner)

lengths=np.linalg.norm(np.roll(xyz,-1,axis=0)-xyz,axis=1)
data={'reference':'../100909998.avif','reconstruction':'hand-traced perspective approximation; synthetic smooth elevation, not surveyed',
      'lap_length_3d_m':float(lengths.sum()),'width_m':width,'white_line_width_m':.06,
      'elevation_range_m':float(np.ptp(z)),'constant_speed_target_mps':5,'ideal_lap_time_s':72,
      'ue_origin_cm':[15000,0,0],'coordinate_system':'x forward, y left, z up; UE x=15000+100*x,y=-100*y,z=100*z',
      'centerline_m':xyz.tolist(),'left_boundary_m':left.tolist(),'right_boundary_m':right.tolist(),
      'station_m':np.r_[0,np.cumsum(lengths[:-1])].tolist(),
      'spawn_ue_cm':[15000,0,float(z[0]*100+3)],
      'spawn_yaw_ue_deg':float(-np.degrees(np.arctan2(tangent[0,1],tangent[0,0])))}
(OUT/'track.json').write_text(json.dumps(data,indent=2)+'\n')
print(json.dumps({k:v for k,v in data.items() if not isinstance(v,list)},indent=2))
print('Bounds m:',xyz.min(axis=0),xyz.max(axis=0))
