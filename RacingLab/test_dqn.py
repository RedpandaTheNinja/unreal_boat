import json,tempfile,unittest
from pathlib import Path
import numpy as np
import torch
from unittest.mock import patch
from dqn_agent import Agent
from dqn_train import validate_config
from unreal_dqn_env import UnrealEnv,Track,ROOT

class DQNTests(unittest.TestCase):
    def cfg(self):
        cfg=json.loads((Path(__file__).parent/'dqn_settings.json').read_text())
        return dict(cfg,hidden_units=16,batch_size=4,learning_starts=4,target_update_steps=2)
    def test_learning_target_and_resume(self):
        a=Agent(3,2,self.cfg());old=[p.clone() for p in a.online.parameters()];target=[p.clone() for p in a.target.parameters()]
        for i in range(4):a.remember([1,0,0],0,1,[0,0,0],True,.1)
        self.assertTrue(np.isfinite(a.learn()))
        self.assertTrue(any(not torch.equal(x,y) for x,y in zip(old,a.online.parameters())))
        self.assertTrue(all(torch.equal(x,y) for x,y in zip(target,a.target.parameters())))
        a.learn();self.assertTrue(all(torch.equal(x,y) for x,y in zip(a.online.parameters(),a.target.parameters())))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.pt';a.save(path,'test');b=Agent(3,2,self.cfg());b.load(path,'test')
            self.assertEqual((b.steps,b.updates,len(b.replay)),(4,2,4))
            self.assertEqual(a.act([1,0,0],False),b.act([1,0,0],False))
            with self.assertRaises(ValueError):b.load(path,'wrong')
    def test_sensor_observation(self):
        env=UnrealEnv.__new__(UnrealEnv);env.track=Track(ROOT/'track/generated/track.json');env.points=np.asarray(env.track.points);env.vehicle=dict(speed_mps=3)
        p=json.loads((ROOT/'car_model/bump_substep.json').read_text())[20]
        obs,g=env.observe(p);self.assertEqual(len(obs),74);self.assertTrue(np.isfinite(obs).all());self.assertGreater(g['left_margin'],0)
        p['front_scan']['ranges_m']=[None]*360
        obs,_=env.observe(p)
        self.assertTrue(np.allclose(obs[26:50:2],1));self.assertTrue(np.allclose(obs[27:50:2],0))
    def test_config(self):
        c=self.cfg();validate_config(c)
        for k,v in [('episodes',0),('batch_size',50000),('gamma',2),('decision_seconds',1)]:
            with self.assertRaises(ValueError):validate_config(dict(c,**{k:v}))
    def test_reset_ignores_old_brake_reply(self):
        import copy
        base=json.loads((ROOT/'car_model/bump_substep.json').read_text())[20]
        class Client:
            def __init__(self):self.now=0.;self.token='old';self.checked=False
            def request(self,command,accept):
                self.now+=.2;p=copy.deepcopy(base);p.update(time_s=self.now,episode_api_version=1,reset_token=self.token,forward_speed_mps=0.)
                if 'reset_episode' in command:
                    assert not accept(p),'A stale brake reply must not satisfy reset'
                    self.checked=True;self.token=command['reset_episode'];p['reset_token']=self.token
                for k in ['imu','front_scan','rear_scan']:p[k]['stamp_s']=self.now
                if 'configure_vehicle' in command:p.update(command['configure_vehicle'],vehicle_config_version=1,configuration_accepted=True)
                assert accept(p)
                return p
        env=UnrealEnv.__new__(UnrealEnv);env.track=Track(ROOT/'track/generated/track.json');env.points=np.asarray(env.track.points)
        env.vehicle=dict(speed_mps=1,max_speed_mps=138.5824,tire_grip_multiplier=1);env.client=Client()
        with patch('unreal_dqn_env.time.sleep',return_value=None):obs,_,_=env.reset()
        self.assertTrue(env.client.checked);self.assertEqual(len(obs),74)
if __name__=='__main__':unittest.main()
