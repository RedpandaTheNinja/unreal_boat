"""Train/evaluate a sensor-vector Double DQN against the live Unreal F1."""
import argparse,hashlib,json,math,time
from pathlib import Path
from dqn_agent import Agent
from unreal_dqn_env import UnrealEnv
from atomic_files import replace_retry

HERE=Path(__file__).resolve().parent

def validate_config(c):
    for k in ['episodes','max_episode_steps','hidden_units','batch_size','replay_capacity','learning_starts','target_update_steps','epsilon_decay_steps','evaluation_every_episodes']:
        if type(c[k]) is not int or c[k]<1: raise ValueError(f'{k} must be a positive integer')
    for k in ['max_episode_seconds','decision_seconds','learning_rate','gamma','epsilon_start','epsilon_end']:
        if not math.isfinite(c[k]): raise ValueError(f'{k} must be finite')
    if not .05<=c['decision_seconds']<=.2 or c['max_episode_seconds']<=0: raise ValueError('decision_seconds must be [.05,.2]; duration must be positive')
    if not 0<c['learning_rate']<=.1 or not 0<c['gamma']<=1: raise ValueError('Invalid learning rate or gamma')
    if not 0<=c['epsilon_end']<=c['epsilon_start']<=1: raise ValueError('Invalid exploration schedule')
    if c['replay_capacity']<max(c['batch_size'],c['learning_starts']): raise ValueError('Replay capacity smaller than learning warmup/batch')
    if not c['speed_fractions'] or any(not math.isfinite(v) or not 0<v<=1 for v in c['speed_fractions']): raise ValueError('speed_fractions must be in (0,1]')
    if not c['steering_actions_rad'] or any(not math.isfinite(v) or abs(v)>.462 for v in c['steering_actions_rad']): raise ValueError('Steering actions outside physical limit')

def atomic_json(path,data,best_effort=False):
    path=Path(path);tmp=path.with_name(path.name+f'.{time.time_ns()}.tmp')
    try:
        tmp.write_text(json.dumps(data,indent=2,allow_nan=False));replace_retry(tmp,path)
    except PermissionError:
        if not best_effort:raise
    finally:
        try:tmp.unlink(missing_ok=True)
        except OSError:pass

def signature(env,c):
    data=dict(observation='groundtruth_lidar_imu_v1_74',actions=env.actions,vehicle=env.vehicle,hidden=c['hidden_units'],decision_seconds=c['decision_seconds'],
        track=hashlib.sha256((HERE.parent/'track/generated/track.json').read_bytes()).hexdigest())
    # Display/old tabular tuning is not part of this policy's environment contract.
    data['vehicle']={k:env.vehicle[k] for k in ['speed_mps','max_speed_mps','tire_grip_multiplier']}
    return hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()

def episode(env,agent,training,folder,signature_value):
    obs,p,g=env.reset();start=p['time_s'];rows=[];total=0.;losses=[];reason='time_limit';finished=False
    last_print=time.monotonic();last_status=0.
    try:
        for step in range(env.cfg['max_episode_steps']):
            action=agent.act(obs,training)
            next_obs,r,terminated,info=env.step(action)
            if training:
                agent.remember(obs,action,r,next_obs,terminated,info['dt']);loss=agent.learn()
                if loss is not None: losses.append(loss)
            total+=r;rows.append(dict(info,action=action,reward=r));obs=next_obs
            elapsed=info['packet']['time_s']-start
            status=dict(mode='train' if training else 'evaluate',episode=agent.episodes+1 if training else agent.episodes,
                episode_step=step+1,total_training_steps=agent.steps,gradient_updates=agent.updates,epsilon=agent.epsilon() if training else 0,
                progress_m=info['progress_m'],reward=total,loss=losses[-1] if losses else None,elapsed_sim_s=elapsed)
            if time.monotonic()-last_status>.5 or terminated:
                atomic_json(folder/'live_status.json',status,best_effort=True);last_status=time.monotonic()
            if time.monotonic()-last_print>=5:
                print(json.dumps(status),flush=True);last_print=time.monotonic()
            if terminated: reason=info['reason'];finished=True;break
            if elapsed>=env.cfg['max_episode_seconds']: finished=True;break
        else: reason='step_limit';finished=True
    except BaseException as exc:
        atomic_json(folder/f'{time.time_ns()}_aborted.json',dict(error=str(exc),rows=rows,training=training,signature=signature_value))
        raise
    finally: env.stop()
    summary=dict(mode='train' if training else 'evaluate',training_episode=agent.episodes+1 if training else agent.episodes,
        steps=len(rows),total_training_steps=agent.steps,gradient_updates=agent.updates,total_reward=total,
        termination=reason,elapsed_sim_s=rows[-1]['packet']['time_s']-start if rows else 0,
        progress_m=env.progress,lap_complete=reason=='lap_complete',valid=finished and reason in ['lap_complete','time_limit','step_limit'],
        mean_loss=sum(losses)/len(losses) if losses else None,signature=signature_value)
    name=f"{time.time_ns()}_{summary['mode']}_ep{summary['training_episode']:04d}.json"
    atomic_json(folder/name,dict(summary=summary,vehicle=env.vehicle,dqn_settings=env.cfg,rows=rows))
    atomic_json(folder/'latest_summary.json',summary);print(json.dumps(summary),flush=True)
    return summary,rows

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['train','evaluate']);p.add_argument('--episodes',type=int);p.add_argument('--seconds',type=float)
    p.add_argument('--fresh',action='store_true');p.add_argument('--checkpoint',type=Path,default=HERE/'dqn_models/latest.pt');args=p.parse_args()
    if args.fresh and args.mode!='train': p.error('--fresh is for training only')
    vehicle=json.loads((HERE/'settings.json').read_text());cfg=json.loads((HERE/'dqn_settings.json').read_text())
    if args.episodes is not None:cfg['episodes']=args.episodes
    if args.seconds is not None:cfg['max_episode_seconds']=args.seconds
    validate_config(cfg);env=UnrealEnv(vehicle,cfg);folder=HERE/'dqn_runs';folder.mkdir(exist_ok=True)
    agent=None;ready=False;sig=signature(env,cfg)
    try:
        obs,_,_=env.reset();agent=Agent(len(obs),len(env.actions),cfg)
        if args.checkpoint.exists() and not args.fresh:agent.load(args.checkpoint,sig)
        elif args.mode=='evaluate':raise RuntimeError('No DQN checkpoint; run training first')
        if args.fresh and args.checkpoint.exists():
            args.checkpoint.rename(args.checkpoint.with_name(f'archived_{time.time_ns()}.pt'))
        ready=True
        print(f'Double DQN: {len(obs)} inputs, {len(env.actions)} direct speed/steering actions. Resume episode {agent.episodes}, updates {agent.updates}.',flush=True)
        def evaluate():
            summary,rows=episode(env,agent,False,folder,sig)
            path=folder/'best_evaluation.json';old=json.loads(path.read_text()) if path.exists() else None
            protocol=dict(signature=sig,seconds=cfg['max_episode_seconds'],max_steps=cfg['max_episode_steps'])
            def score(s):return (int(s['lap_complete']),-s['elapsed_sim_s'] if s['lap_complete'] else s['progress_m'])
            if summary['valid'] and (old is None or old.get('protocol')!=protocol or score(summary)>score(old['summary'])):
                atomic_json(path,dict(protocol=protocol,summary=summary,rows=rows))
                agent.save(HERE/'dqn_models/best.pt',sig);env.show_line(rows)
                print('Green Unreal overlay updated: '+('best completed lap' if summary['lap_complete'] else 'best valid PARTIAL evaluation; not a completed racing lap'),flush=True)
            elif old and old.get('protocol')==protocol:env.show_line(old['rows'])
        for _ in range(cfg['episodes'] if args.mode=='train' else (args.episodes or 1)):
            if args.mode=='evaluate':evaluate();continue
            episode(env,agent,True,folder,sig);agent.episodes+=1
            agent.save(args.checkpoint,sig)
            if agent.episodes%cfg['evaluation_every_episodes']==0:
                agent.save(HERE/f'dqn_models/episode_{agent.episodes:05d}.pt',sig);evaluate()
        if args.mode=='train':evaluate()
    except KeyboardInterrupt:print('Interrupted; saving learner and braking.',flush=True)
    finally:
        env.close()
        if ready and agent is not None and args.mode=='train':agent.save(args.checkpoint,sig)

if __name__=='__main__':main()
