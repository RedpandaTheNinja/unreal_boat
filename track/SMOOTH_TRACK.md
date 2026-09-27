# Smooth circuit and recoverable runoff

Applied to `/Game/ThirdPerson/Lvl_ThirdPerson` on 2026-09-19. New mesh assets are under `/Game/Fable/ReferenceTrackSmooth`.

## Geometry

- Road remains 1.20 m wide, with smooth shared normals and approximately 5 cm longitudinal mesh spacing. Imported broad convex hulls were removed; collision uses the surface triangles.
- White edge paint stays inside the road and has no collision.
- Alternating red/white curb stones lie outside the paint. They are continuous rounded ramps, normally 12 cm wide and at most 6 mm high, with 30 cm colour segments. At narrowly separated hairpins, curb width tapers to as little as 1.64 cm and height reduces proportionally; this avoids overlapping surfaces and steep miniature steps.
- A conforming terrain mesh joins the outside curb edges at identical heights. It fills the infield and surrounding ground and extends approximately 18 m beyond the circuit bounds, blending to the existing base ground. No vertical bank faces are used. Every curb boundary segment was checked for exactly one adjacent terrain triangle; terrain edges are manifold.
- Tight bends were locally faired by at most 3.43 cm in plan. Nearby road sections with conflicting elevations were blended by at most 15.27 cm vertically to avoid a steep slit between adjacent hairpins. The broad hill profile remains.
- Revised centreline length is 359.971 m, approximately 72 seconds at a hypothetical constant 5 m/s. This is geometry, not a measured lap time.
- 99% of sampled terrain triangles within 2 m of the centreline have grades below 19.4%; the steepest terrain triangle overall is about 43.6% (23.6 degrees). The terrain is continuous, but driving at arbitrary speed or in every off-track location is not validated.

`build_smooth_track.py` generates the meshes and geometric checks using the existing simulation virtual environment. Its immutable reference is `generated_smooth/reference_before_smoothing.json`. It writes the revised route to `generated_smooth/track.json`; the active dashboard/controller route was copied to `generated/track.json`. The dashboard server was restarted to load that revision.

```powershell
& ./simulation_ml/.venv/Scripts/python.exe track/build_smooth_track.py
```

Generation does not automatically reimport assets into Unreal. When importing, use unit actor scale, rotation zero and location (15000,0,0) cm. Use Complex-as-Simple collision for road, curbs and terrain, then remove auto-generated simple collision shapes and save. Paint components must have NoCollision. OBJ files include UVs, normals and smoothing information.

## Validation

`validate_runoff.py` commanded a bounded low-speed steering excursion followed by reversing along the same turn. It passed on both sides:

| Test | Maximum distance from centreline | Final distance | Final speed | Final wheel contacts |
|---|---:|---:|---:|---|
| Left curb/runoff | 1.033 m | 0.067 m | 0 m/s | All four |
| Right curb/runoff | 1.058 m | -0.068 m | 0 m/s | All four |

Evidence: `generated_smooth/runoff_left_result.json`, `runoff_right_result.json` and corresponding packet files. The test checked upright orientation throughout. These are local low-speed recovery checks near the start, not full-lap or 5 m/s validation. A startup jolt was observed with the previous partially compressed placement; the saved F1 start was moved to (15000,0,261.17) cm, pitch 1.34°, yaw -46.08965634°, approximately full suspension extension above the road.

The disabled original vertical-bank actors remain in `ReferenceTrack_ArchivedBanks`, with visibility and collision disabled. Original mesh assets and the original track JSON are retained. The original flat base ground remains beneath the new contoured surface.
