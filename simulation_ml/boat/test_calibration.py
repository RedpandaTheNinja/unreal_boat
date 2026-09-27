import csv,tempfile,unittest
from pathlib import Path
import numpy as np
from calibrate_surge import fit,design
class CalibrationTests(unittest.TestCase):
 def make(self,p,phase=0):
  t=np.arange(0,30,.02);v=.9+np.sin(t*.7+phase)*.65+np.sin(t*1.7)*.12;a=.455*np.cos(t*.7+phase)+.204*np.cos(t*1.7)
  f=190*a+25*v+45*v*np.abs(v)
  with p.open('w',newline='') as out:
   w=csv.writer(out);w.writerow(['time_s','speed_mps','thrust_n']);w.writerows(zip(t,v,f))
 def test_recovers_mass_drag_on_independent_run(self):
  with tempfile.TemporaryDirectory() as td:
   a,b=Path(td)/'a.csv',Path(td)/'b.csv';self.make(a);self.make(b,.4);r=fit(a,b,172.3651006)
   self.assertAlmostEqual(r['surge_effective_mass_kg'],190,delta=.1);self.assertAlmostEqual(r['surge_linear_drag'],25,delta=.1);self.assertAlmostEqual(r['surge_quadratic_drag'],45,delta=.1);self.assertLess(r['held_out_force_rmse_n'],.1)
 def test_rejects_unexcited_data(self):
  with tempfile.TemporaryDirectory() as td:
   p=Path(td)/'constant.csv';p.write_text('time_s,speed_mps,thrust_n\n'+''.join(f'{i},1,70\n' for i in range(30)))
   with self.assertRaises(ValueError):design(p)
if __name__=='__main__':unittest.main()
