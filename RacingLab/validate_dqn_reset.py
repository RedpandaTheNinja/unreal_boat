"""Live idempotency and settled-pose reset checks; requires Play and new plugin."""
import json,math
from pathlib import Path
from unreal_dqn_env import UnrealEnv
HERE=Path(__file__).resolve().parent
env=UnrealEnv(json.loads((HERE/'settings.json').read_text()),json.loads((HERE/'dqn_settings.json').read_text()))
rows=[]
try:
    for i in range(3):
        obs,p,g=env.reset();token=p['reset_token'];episode=p['episode_id']
        duplicate=env.packet(dict(reset_episode=token,speed_mps=0,steering_rad=0))
        assert duplicate['episode_id']==episode,'Duplicate token caused another reset'
        rows.append(dict(episode_id=episode,position=[p['x_m'],p['y_m'],p['z_m']],contacts=[w['in_contact'] for w in p['wheels']],observation_size=len(obs)))
    for row in rows[1:]:assert math.dist(row['position'],rows[0]['position'])<.02,'Settled reset position drift'
    result=dict(passed=True,resets=rows,idempotent=True)
    (HERE/'dqn_runs/reset_acceptance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:env.close()
