"""Embed the existing seed-0 CSV into the inline replay; no simulation rerun."""
import csv
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
out = root / 'test-results/current-car'
scene = json.loads((root / 'unreal_fable/specs/worlds/car_track_oval.json').read_text())
with (out / 'car_track_oval_0.csv').open() as stream:
    records = list(csv.DictReader(stream))
selected = records[::10]
if selected[-1] is not records[-1]:
    selected.append(records[-1])
payload = {'track': scene['track']['centerline_m'], 'width': scene['track']['width_m'],
           'rows': [[round(float(r[k]), 5) for k in ('t', 'x', 'y', 'yaw')] for r in selected]}
target = out / 'oval-replay.html'
html = target.read_text(encoding='utf-8')
start = html.index('>', html.index('<script type="application/json" id="oval-data"')) + 1
end = html.index('</script>', start)
target.write_text(html[:start] + json.dumps(payload, separators=(',', ':')) + html[end:], encoding='utf-8')
assert len(payload['rows']) > 300
assert payload['rows'][-1][0] == 71.6
assert target.stat().st_size < 1_000_000
print(f'Embedded {len(selected)} samples; final time {payload["rows"][-1][0]} s')
