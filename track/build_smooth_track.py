"""Smooth road, low rounded curbs and a conforming continuous terrain mesh.
Uses the existing track.json as the reference; leaves its route/telemetry frame intact.
OBJ is centimetres in X-forward/Y-left/Z-up; Unreal OBJ import reflects Y.
"""
from pathlib import Path
import json
import numpy as np
from scipy.interpolate import CubicSpline, RBFInterpolator
from scipy.spatial import Delaunay, cKDTree
from scipy.ndimage import gaussian_filter1d
from scipy.sparse import coo_matrix, eye
from scipy.sparse.linalg import spsolve

OUT=Path(__file__).parent/'generated_smooth'
OUT.mkdir(exist_ok=True)
original=OUT/'reference_before_smoothing.json'
if not original.exists():original.write_bytes((Path(__file__).parent/'generated/track.json').read_bytes())
source=json.loads(original.read_text())
p=np.array(source['centerline_m']); n=len(p)
s=np.r_[0,np.cumsum(np.linalg.norm(np.roll(p,-1,axis=0)-p,axis=1))]
curve=CubicSpline(s,np.vstack([p,p[0]]),bc_type='periodic')
station=np.linspace(0,s[-1],int(np.ceil(s[-1]/.05)),endpoint=False)
road=curve(station)
original_xy=road[:,:2].copy()
for _ in range(300):
    velocity=(np.roll(road[:,:2],-1,axis=0)-np.roll(road[:,:2],1,axis=0))/2
    acceleration=np.roll(road[:,:2],-1,axis=0)-2*road[:,:2]+np.roll(road[:,:2],1,axis=0)
    curvature=np.abs(velocity[:,0]*acceleration[:,1]-velocity[:,1]*acceleration[:,0])/np.maximum(np.linalg.norm(velocity,axis=1)**3,1e-12)
    if curvature.max()<1/.95:break
    weights=np.minimum(3*gaussian_filter1d((curvature>1/.95).astype(float),6,mode='wrap'),1)
    road[:,:2]+=.4*weights[:,None]*(gaussian_filter1d(road[:,:2],3,axis=0,mode='wrap')-road[:,:2])
tangent=np.roll(road[:,:2],-1,axis=0)-np.roll(road[:,:2],1,axis=0); tangent/=np.linalg.norm(tangent,axis=1)[:,None]
normal=np.c_[-tangent[:,1],tangent[:,0]]; N=len(road)
tree=cKDTree(road[:,:2])
distances,indices=tree.query(road[:,:2],k=256)
separation=np.abs(indices-np.arange(N)[:,None]);separation=np.minimum(separation,N-separation)
distances[separation<80]=np.inf
nearby=distances.min(axis=1)
neighbor=indices[np.arange(N),distances.argmin(axis=1)]
# Closely adjacent hairpins cannot retain conflicting heights and also have
# gentle ground between them. Fair those local elevations with a sparse solve.
ii=np.arange(N); jj=(ii+1)%N; kk=(ii-1)%N
D2=coo_matrix((np.r_[np.ones(N),-2*np.ones(N),np.ones(N)],
              (np.tile(ii,3),np.r_[kk,ii,jj])),shape=(N,N)).tocsr()
weight=20*np.maximum(0,2.5-nearby)**2/np.maximum(nearby-1.2,.03)**2
weight[~np.isfinite(weight)]=0
L=coo_matrix((np.r_[weight,weight,-weight,-weight],
             (np.r_[ii,neighbor,ii,neighbor],np.r_[ii,neighbor,neighbor,ii])),shape=(N,N)).tocsr()
old_z=road[:,2].copy()
road[:,2]=spsolve(eye(N,format='csr')+10000*(D2.T@D2)+L,old_z)
outer_offset=np.minimum(.72,np.maximum(.605,nearby/2-.02))
outer_offset=np.minimum(outer_offset,gaussian_filter1d(outer_offset,8,mode='wrap'))
curb_width=outer_offset-.6
def ring(offset,height=0):
    q=road.copy();q[:,:2]+=np.asarray(offset)[...,None]*normal;q[:,2]+=height;return q

def faces_ribbon(rows):
    k=len(rows); vertices=np.stack(rows,axis=1).reshape(-1,3); faces=[]
    for i in range(N):
        j=(i+1)%N
        for a in range(k-1): faces.extend([(i*k+a,i*k+a+1,j*k+a+1),(i*k+a,j*k+a+1,j*k+a)])
    return vertices,np.array(faces,dtype=int)

def save(name,vertices,faces):
    # Orient all driveable surfaces up and share normals across triangulation.
    f=faces.copy(); tri=vertices[f]; cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    wrong=cross[:,2]<0;f[wrong]=f[wrong][:,[0,2,1]]
    tri=vertices[f];cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    assert np.min(np.linalg.norm(cross,axis=1))>1e-10, name
    normals=np.zeros_like(vertices)
    for j in range(3):np.add.at(normals,f[:,j],cross)
    lengths=np.linalg.norm(normals,axis=1);normals[lengths>0]/=lengths[lengths>0,None]
    lines=['o '+name,'s 1']
    lines+=['v %.7f %.7f %.7f'%tuple(v*100) for v in vertices]
    lines+=['vt %.7f %.7f'%tuple(v[:2]) for v in vertices]
    lines+=['vn %.8f %.8f %.8f'%tuple(v) for v in normals]
    lines+=['f '+' '.join(f'{i+1}/{i+1}/{i+1}' for i in face) for face in f]
    (OUT/(name+'.obj')).write_text('\n'.join(lines)+'\n')
    return dict(vertices=len(vertices),triangles=len(f))

report={}
report['minimum_centerline_radius_m']=float(1/curvature.max())
report['maximum_local_plan_adjustment_m']=float(np.linalg.norm(road[:,:2]-original_xy,axis=1).max())
report['curb_width_min_m']=float(curb_width.min())
report['minimum_nonlocal_center_spacing_m']=float(nearby.min())
report['maximum_elevation_adjustment_m']=float(np.abs(road[:,2]-old_z).max())
report['asphalt']=save('asphalt',*faces_ribbon([ring(x) for x in np.linspace(-.6,.6,7)]))
for name,offsets in [('white_left',[.54,.6]),('white_right',[-.6,-.54])]:
    report[name]=save(name,*faces_ribbon([ring(x,.0005) for x in offsets]))
for side in (-1,1):
    rows=[ring(side*(.6+t*curb_width),np.minimum(.006,curb_width*.05)*np.sin(np.pi*t)**2) for t in np.linspace(0,1,9)]
    v,f=faces_ribbon(rows)
    red=(np.floor(station/.30).astype(int)%2==0)
    mask=np.repeat(red,16)
    for color,m in [('red',mask),('white',~mask)]:
        name=f'curb_{"left" if side>0 else "right"}_{color}'
        report[name]=save(name,v,f[m])

boundary=np.vstack([ring(-outer_offset),ring(outer_offset)])
lo=road[:,:2].min(axis=0)-18;hi=road[:,:2].max(axis=0)+18
def grid(step):
    x,y=np.meshgrid(np.arange(lo[0],hi[0]+step/2,step),np.arange(lo[1],hi[1]+step/2,step))
    return np.c_[x.ravel(),y.ravel()]
fine=grid(.25);dist,_=tree.query(fine);fine=fine[(dist>.80)&(dist<3)]
coarse=grid(1.0);dist,_=tree.query(coarse);coarse=coarse[dist>=3]
segments=[(side*N+i,side*N+(i+1)%N) for side in range(2) for i in range(N)]
for attempt in range(12):
    xy=np.vstack([boundary[:,:2],fine,coarse]);tri=Delaunay(xy)
    edges=set()
    for a,b,c in tri.simplices:
        for i,j in ((a,b),(b,c),(c,a)):edges.add((min(i,j),max(i,j)))
    missing=[(a,b) for a,b in segments if (min(a,b),max(a,b)) not in edges]
    if attempt==0 and missing:
        print('Missing boundary segments:',[(a,b,boundary[a].tolist(),boundary[b].tolist()) for a,b in missing],flush=True)
        aa=boundary[np.array(segments)[:,0],:2];bb=boundary[np.array(segments)[:,1],:2];vv=bb-aa
        cross2=lambda a,b:a[...,0]*b[...,1]-a[...,1]*b[...,0]
        for a,b in missing:
            r=boundary[b,:2]-boundary[a,:2];den=cross2(r,vv);delta=aa-boundary[a,:2]
            with np.errstate(divide='ignore',invalid='ignore'):
                tt=cross2(delta,vv)/den;uu=cross2(delta,r)/den
            hits=np.where((tt>1e-7)&(tt<1-1e-7)&(uu>1e-7)&(uu<1-1e-7))[0]
            print('Crossing',a,b,[(segments[k],float(tt[k]),float(uu[k])) for k in hits],flush=True)
    if not missing:break
    for a,b in missing:
        k=len(boundary);boundary=np.vstack([boundary,(boundary[a]+boundary[b])/2])
        segments.remove((a,b));segments.extend([(a,k),(k,b)])
assert not missing, f'Terrain triangulation did not preserve {len(missing)} curb boundary edges'
centroids=xy[tri.simplices].mean(axis=1)
d,ci=tree.query(centroids)
faces=tri.simplices[d>outer_offset[ci]]
edge_counts={}
for a,b,c in faces:
    for i,j in ((a,b),(b,c),(c,a)):
        key=(min(i,j),max(i,j));edge_counts[key]=edge_counts.get(key,0)+1
assert all(edge_counts.get((min(a,b),max(a,b)))==1 for a,b in segments), 'Open or overlapping terrain/curb seam'
assert max(edge_counts.values())==2, 'Non-manifold terrain'

# Smooth global interpolant: road/curb elevation constraints plus a low outer rim.
# Matching both .6/.72 edges keeps near-road crossfall mild; exact seam heights
# are assigned below, so there is no gap even between interpolation stations.
sample=np.arange(0,N,20)
anchors=np.vstack([ring(x)[sample] for x in (-outer_offset,-.6,0,.6,outer_offset)])
outer=[]
for t in np.linspace(0,1,65):
    outer.extend([[lo[0]+t*(hi[0]-lo[0]),lo[1],0],
                  [lo[0]+t*(hi[0]-lo[0]),hi[1],0],
                  [lo[0],lo[1]+t*(hi[1]-lo[1]),0],
                  [hi[0],lo[1]+t*(hi[1]-lo[1]),0]])
anchors=np.unique(np.vstack([anchors,outer]),axis=0)
surface=RBFInterpolator(anchors[:,:2],anchors[:,2],kernel='thin_plate_spline',smoothing=1e-7)
heights=np.concatenate([surface(batch) for batch in np.array_split(xy,32)])
residual=boundary[:,2]-heights[:len(boundary)]
bd,bi=cKDTree(boundary[:,:2]).query(xy,k=2)
bw=1/np.maximum(bd,1e-10);bw/=bw.sum(axis=1)[:,None]
heights+=(residual[bi]*bw).sum(axis=1)*np.exp(-(bd[:,0]/.8)**2)
heights[:len(boundary)]=boundary[:,2]
terrain=np.c_[xy,heights]
report['terrain']=save('terrain',terrain,faces)
norm=np.cross(terrain[faces[:,1]]-terrain[faces[:,0]],terrain[faces[:,2]]-terrain[faces[:,0]])
grade=np.linalg.norm(norm[:,:2],axis=1)/np.abs(norm[:,2])
near=tree.query(terrain[faces].mean(axis=1)[:,:2])[0]<2.0
report.update(road_station_step_m=float(s[-1]/N),road_width_m=1.2,curb_width_m=.12,curb_height_m=.006,
              boundary_seams_preserved=True,terrain_max_grade=float(grade.max()),
              terrain_near_track_grade_p99=float(np.percentile(grade[near],99)),
              terrain_z_range_m=[float(heights.min()),float(heights.max())],
              steepest_terrain_triangle_center_m=terrain[faces[np.argmax(grade)]].mean(axis=0).tolist(),
              ue_origin_cm=[15000,0,0],actor_rotation_deg=[0,0,0])
np.savez_compressed(OUT/'geometry.npz',terrain_vertices=terrain,terrain_faces=faces,road=road,normal=normal,station=station)
lengths=np.linalg.norm(np.roll(road,-1,axis=0)-road,axis=1)
updated=dict(source,centerline_m=road.tolist(),left_boundary_m=ring(.6).tolist(),right_boundary_m=ring(-.6).tolist(),
             station_m=np.r_[0,np.cumsum(lengths[:-1])].tolist(),lap_length_3d_m=float(lengths.sum()),
             ideal_lap_time_s=float(lengths.sum()/5),surface_revision='smooth_road_rounded_curbs_continuous_terrain',
             elevation_range_m=float(np.ptp(road[:,2])))
(OUT/'track.json').write_text(json.dumps(updated,indent=2)+'\n')
report['lap_length_3d_m']=float(lengths.sum())
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
