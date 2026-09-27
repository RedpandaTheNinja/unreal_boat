"""Fit straight-line effective mass and drag using independently measured thrust.
CSV: time_s, speed_mps, thrust_n. Use separate --validation run, calm water,
minimal yaw, same load, and measured effective propulsive force (not nameplate thrust).
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear

def design(path):
 with Path(path).open(newline='') as f: rows=list(csv.DictReader(f))
 a=np.array([[float(r[k]) for k in ('time_s','speed_mps','thrust_n')] for r in rows])
 if len(a)<20 or not np.isfinite(a).all() or np.any(np.diff(a[:,0])<=0):raise ValueError('Need >=20 finite, strictly time-ordered samples')
 t,v,f=a.T
 acceleration=np.gradient(v,t,edge_order=2)
 x=np.column_stack((acceleration,v,v*np.abs(v)))[2:-2];f=f[2:-2]
 if np.linalg.matrix_rank(x)<3 or np.linalg.cond(x)>1e5:raise ValueError('Insufficient excitation: include acceleration and coast-down over a range of speeds')
 return x,f

def fit(train,validation,mass):
 if mass<=0:raise ValueError('Mass must be positive')
 x,y=design(train)
 result=lsq_linear(x,y,bounds=([mass,0,0],[np.inf,np.inf,np.inf]))
 if not result.success:raise ValueError('Fit failed: '+result.message)
 hold,truth=design(validation);prediction=hold@result.x
 return {'status':'candidate_fit_requires_physical_review','loaded_mass_kg':mass,'surge_effective_mass_kg':float(result.x[0]),'surge_added_mass_kg':float(result.x[0]-mass),'surge_linear_drag':float(result.x[1]),'surge_quadratic_drag':float(result.x[2]),'training_force_rmse_n':float(np.sqrt(np.mean((x@result.x-y)**2))),'held_out_force_rmse_n':float(np.sqrt(np.mean((prediction-truth)**2))),'training_file':str(train),'validation_file':str(validation),'note':'Surge only. Sway, yaw, thrust curves, wave response and payload trim need separate measurements. No parameters are applied automatically.'}
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--training',required=True,type=Path);p.add_argument('--validation',required=True,type=Path);p.add_argument('--mass-kg',type=float,default=172.3651006);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.training.resolve()==a.validation.resolve():p.error('Validation must be a different measured run')
 report=fit(a.training,a.validation,a.mass_kg);a.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
