"""Compare recorded chassis motion against the generated road, without altering physics."""
import json, math, statistics, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def analyze(path):
    rows=json.loads(Path(path).read_text()); track=json.loads((ROOT/'track/generated/track.json').read_text())
    points=track['centerline_m']; heights=[]; az=[]; offsets=[]; steps=[]
    for r in rows:
        x=r['x_m']-50; y=r['y_m']
        i=min(range(len(points)),key=lambda i:(points[i][0]-x)**2+(points[i][1]-y)**2)
        heights.append(r['z_m']-points[i][2]-.02)
        az.append(r['imu']['linear_acceleration_mps2']['z'])
        offsets.extend(w['suspension_offset_m'] for w in r['wheels'])
    steps=[b['time_s']-a['time_s'] for a,b in zip(rows,rows[1:])]
    result=dict(samples=len(rows),all_wheels_contact_fraction=sum(all(w['in_contact'] for w in r['wheels']) for r in rows)/len(rows),
        approximate_hull_clearance_min_m=min(heights),approximate_hull_clearance_max_m=max(heights),
        vertical_specific_force_min_mps2=min(az),vertical_specific_force_max_mps2=max(az),vertical_specific_force_std_mps2=statistics.pstdev(az),
        suspension_offset_range_m=[min(offsets),max(offsets)],sample_dt_max_s=max(steps),
        note='Clearance uses closest centreline elevation and chassis bottom -2 cm, not a collision contact measurement; valid near centreline only.')
    return result
if __name__=='__main__':
    print(json.dumps(analyze(sys.argv[1]),indent=2))
