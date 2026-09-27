"""Original 60 x 25 cm F1-inspired prototype. UE cm, X forward/Y right/Z up.
OBJ import reflects Y, so export reflected Y and reverse face winding.
Every face has explicit normals and non-degenerate UVs.
"""
from pathlib import Path
import math
import json

OUT = Path(__file__).parent / 'generated_f1'
OUT.mkdir(exist_ok=True)

def sub(a,b): return tuple(x-y for x,y in zip(a,b))
def cross(a,b): return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
def dot(a,b): return sum(x*y for x,y in zip(a,b))
def unit(a):
    d=math.sqrt(dot(a,a)); assert d>1e-9
    return tuple(x/d for x in a)

class Mesh:
    def __init__(self,name): self.name,self.tris=name,[]
    def face(self,*pts):
        for i in range(1,len(pts)-1): self.tris.append((pts[0],pts[i],pts[i+1]))
    def box(self,center,size):
        x,y,z=center; a,b,c=[v/2 for v in size]
        p=[(x+sx*a,y+sy*b,z+sz*c) for sz in (-1,1) for sy in (-1,1) for sx in (-1,1)]
        for q in [(0,2,3,1),(4,5,7,6),(0,1,5,4),(2,6,7,3),(0,4,6,2),(1,3,7,5)]: self.face(*(p[i] for i in q))
    def loft(self,sections,ycenter=0):
        rings=[]
        for x,w,low,high in sections:
            z=(low+high)/2; h=(high-low)/2
            rings.append([(x,ycenter+w*y,z+h*v) for y,v in [(-.7,-1),(.7,-1),(1,-.5),(1,.5),(.7,1),(-.7,1),(-1,.5),(-1,-.5)]])
        self.face(*reversed(rings[0]));self.face(*rings[-1])
        for a,b in zip(rings,rings[1:]):
            for i in range(8): j=(i+1)%8;self.face(a[i],a[j],b[j],b[i])
    def tube(self,a,b,r,n=12):
        axis=unit(sub(b,a)); u=unit(cross(axis,(0,0,1) if abs(axis[2])<.9 else (1,0,0)));v=cross(axis,u)
        rings=[ [tuple(p[k]+r*(u[k]*math.cos(t*2*math.pi/n)+v[k]*math.sin(t*2*math.pi/n)) for k in range(3)) for t in range(n)] for p in (a,b)]
        self.face(*reversed(rings[0]));self.face(*rings[1])
        for i in range(n): j=(i+1)%n;self.face(rings[0][i],rings[0][j],rings[1][j],rings[1][i])
    def save(self):
        lines=['o '+self.name,'s 1']
        for tri in self.tris:
            a,b,c=[(p[0],-p[1],p[2]) for p in (tri[0],tri[2],tri[1])]
            normal=unit(cross(sub(b,a),sub(c,a)));u=unit(sub(b,a));v=cross(normal,u)
            for p in (a,b,c): lines.append('v '+' '.join(f'{t:.7f}' for t in p))
            for p in (a,b,c): lines.append(f'vt {dot(sub(p,a),u)/10:.7f} {dot(sub(p,a),v)/10:.7f}')
            for _ in range(3): lines.append('vn '+' '.join(f'{t:.7f}' for t in normal))
        for i in range(len(self.tris)):
            lines.append('f '+' '.join(f'{j}/{j}/{j}' for j in range(3*i+1,3*i+4)))
        (OUT/(self.name+'.obj')).write_text('\n'.join(lines)+'\n')
        return {'name':self.name,'triangles':len(self.tris)}

parts=[]
body=Mesh('Chassis')
body.loft([(-24,1.8,-1.8,2),(-17,4,-2,4),(-7,5,-2,5),(3,4.3,-1.8,4.8),(11,3,-1.5,3),(23,1.3,-1.3,1.2),(27,.8,-1,0.8)])
for s in (-1,1): body.loft([(-18,1,-1.3,1.7),(-12,2.4,-1.8,3.4),(0,2.5,-1.7,4),(5,1.5,-1.2,3)],s*5.3)
parts.append(body)
floor=Mesh('Floor'); floor.loft([(-26,4,-2.5,-2),(-17,8,-2.5,-2),(5,8,-2.5,-2),(11,3,-2,-1.5),(27,1.1,-1.6,-1.2)])
parts.append(floor)
wing=Mesh('Wings')
for x,z in [(27.8,-1.3),(25.7,-.65)]: wing.box((x,0,z),(2.8,24.5,.35))
for s in (-1,1): wing.box((27,s*12.2,-.3),(6,.6,2.8))
for x,z in [(-27.8,7.5),(-25.8,8.5)]: wing.box((x,0,z),(3,20,.4))
for s in (-1,1): wing.box((-27,s*10,6.8),(6,.5,5));wing.box((-24,s*3.5,3.5),(.7,.5,8))
parts.append(wing)
cockpit=Mesh('Cockpit'); cockpit.loft([(-10,2.6,3.4,5),(-5,3,3.8,5),(0,2.5,3.6,5),(3,1.8,3.4,4.5)])
parts.append(cockpit)
halo=Mesh('Halo')
path=[(4,0,4),(3,0,7),(0,2.6,8),(-5,3,8),(-9,2.4,6)]
for s in (-1,1):
    p=[(x,s*y,z) for x,y,z in path]
    for a,b in zip(p,p[1:]):halo.tube(a,b,.28)
halo.tube((-9,-2.4,6),(-9,2.4,6),.28)
parts.append(halo)
acc=Mesh('Accents')
acc.loft([(5,.45,4.1,4.3),(11,.4,3,3.2),(23,.3,1.2,1.4),(27,.25,.8,1)])
for s in (-1,1):
    acc.box((27.8,s*8,-1.05),(2.4,3,.14));acc.box((-27.8,s*7,7.8),(2.6,3,.15))
    acc.box((-4,s*7.6,3.1),(12,.22,.55))
parts.append(acc)
tire=Mesh('Tire');tire.tube((0,-2.5,0),(0,2.5,0),4.5,48);parts.append(tire)
rim=Mesh('Rim')
for s in (-1,1):
    rim.tube((0,s*2.48,0),(0,s*2.56,0),2.5,32)
parts.append(rim)
hub=Mesh('Hub')
for s in (-1,1):
    hub.tube((0,s*2.57,0),(0,s*2.65,0),.8,16)
    for i in range(8):
        t=i*math.pi/4;hub.tube((math.cos(t)*.9,s*2.6,math.sin(t)*.9),(math.cos(t)*2.25,s*2.6,math.sin(t)*2.25),.14,6)
parts.append(hub)
data=[p.save() for p in parts]
(OUT/'manifest.json').write_text(json.dumps({'dimensions_cm':[60,25,17.5],'wheelbase_cm':37,'wheel_track_cm':20,'wheel_radius_cm':4.5,'parts':data},indent=2)+'\n')
print(json.dumps(data))
