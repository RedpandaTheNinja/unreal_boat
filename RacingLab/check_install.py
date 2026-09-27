"""Offline transfer check: does not command the vehicle."""
import json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
needed=['mcp_gpt.uproject','Content/ThirdPerson/Lvl_ThirdPerson.umap',
 'Plugins/RapyutaSimulationPlugins/Binaries/Win64/UnrealEditor-RapyutaSimulationPlugins.dll',
 'track/generated/track.json','simulation_ml/integrations/rapyuta/rgb_imu_client.py',
 'simulation_ml/integrations/rapyuta/dashboard/server.py']
missing=[p for p in needed if not (ROOT/p).exists()]
cfg=json.loads((ROOT/'RacingLab/settings.json').read_text())
print(json.dumps(dict(project=str(ROOT),python=sys.executable,missing=missing,settings=cfg),indent=2))
sys.exit(bool(missing))
