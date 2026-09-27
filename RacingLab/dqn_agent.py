"""Original PyTorch Double DQN: replay, online/target networks and resumable state."""
import copy,random
from collections import deque
from pathlib import Path
import numpy as np
import torch
from torch import nn
from atomic_files import replace_retry

class Agent:
    def __init__(self,obs_dim,actions,cfg):
        torch.set_num_threads(1); torch.manual_seed(cfg['seed'])
        self.cfg=cfg; self.actions=actions; self.rng=random.Random(cfg['seed'])
        n=cfg['hidden_units']
        self.online=nn.Sequential(nn.Linear(obs_dim,n),nn.ReLU(),nn.Linear(n,n),nn.ReLU(),nn.Linear(n,actions))
        self.target=copy.deepcopy(self.online).eval()
        self.optimizer=torch.optim.Adam(self.online.parameters(),lr=cfg['learning_rate'])
        self.replay=deque(maxlen=cfg['replay_capacity']); self.steps=0; self.updates=0; self.episodes=0
    def epsilon(self):
        c=self.cfg; f=min(1,self.steps/c['epsilon_decay_steps'])
        return c['epsilon_start']+(c['epsilon_end']-c['epsilon_start'])*f
    def act(self,obs,training):
        if training and self.rng.random()<self.epsilon(): return self.rng.randrange(self.actions)
        with torch.no_grad(): return int(self.online(torch.tensor(obs,dtype=torch.float32)).argmax().item())
    def remember(self,s,a,r,ns,terminated,dt):
        # Truncation is not termination: timeout transitions still bootstrap.
        self.replay.append((np.asarray(s,dtype=np.float32),a,float(r),np.asarray(ns,dtype=np.float32),bool(terminated),float(dt)))
        self.steps+=1
    def learn(self):
        c=self.cfg
        if len(self.replay)<max(c['batch_size'],c['learning_starts']): return None
        batch=self.rng.sample(list(self.replay),c['batch_size'])
        s,a,r,ns,done,dt=zip(*batch)
        s=torch.tensor(np.stack(s)); ns=torch.tensor(np.stack(ns))
        a=torch.tensor(a); r=torch.tensor(r); done=torch.tensor(done);dt=torch.tensor(dt)
        q=self.online(s).gather(1,a[:,None]).squeeze(1)
        with torch.no_grad():
            # Double DQN: online chooses; target evaluates.
            best=self.online(ns).argmax(1)
            nxt=self.target(ns).gather(1,best[:,None]).squeeze(1)
            discount=c['gamma']**(dt/c['decision_seconds'])
            target=r+discount*(~done)*nxt
        loss=nn.functional.smooth_l1_loss(q,target)
        if not torch.isfinite(loss): raise RuntimeError('Non-finite DQN loss')
        self.optimizer.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(self.online.parameters(),10.)
        self.optimizer.step(); self.updates+=1
        if self.updates%c['target_update_steps']==0: self.target.load_state_dict(self.online.state_dict())
        return float(loss.item())
    def save(self,path,signature):
        path=Path(path); path.parent.mkdir(parents=True,exist_ok=True);tmp=path.with_suffix('.tmp')
        torch.save(dict(version=1,signature=signature,config=self.cfg,online=self.online.state_dict(),target=self.target.state_dict(),optimizer=self.optimizer.state_dict(),
            steps=self.steps,updates=self.updates,episodes=self.episodes,rng=self.rng.getstate(),torch_rng=torch.get_rng_state(),
            replay=[(s.tolist(),a,r,ns.tolist(),d,dt) for s,a,r,ns,d,dt in self.replay]),tmp)
        replace_retry(tmp,path)
    def load(self,path,signature):
        data=torch.load(path,map_location='cpu',weights_only=True)
        if data['version']!=1 or data['signature']!=signature: raise ValueError('Checkpoint environment/action/observation mismatch. Use --fresh or a separate checkpoint.')
        self.online.load_state_dict(data['online']);self.target.load_state_dict(data['target']);self.optimizer.load_state_dict(data['optimizer'])
        for group in self.optimizer.param_groups: group['lr']=self.cfg['learning_rate']
        self.steps=data['steps'];self.updates=data['updates'];self.episodes=data['episodes'];self.rng.setstate(data['rng']);torch.set_rng_state(data['torch_rng'])
        self.replay.clear()
        for s,a,r,ns,d,dt in data['replay']: self.replay.append((np.asarray(s,dtype=np.float32),a,r,np.asarray(ns,dtype=np.float32),d,dt))
