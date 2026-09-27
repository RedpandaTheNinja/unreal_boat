"""Run inside Unreal's Python editor after enabling the compiled port.

Adds the tandem to the existing oval without deleting the reference car/line.
Writes the loaded original Blueprint body settings for physics review.
"""
import json
from pathlib import Path
import unreal

subsystem = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
level = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
assert level.load_level('/Game/ThirdPerson/Lvl_ThirdPerson')
label = 'Tandem_TurtleBot_Windows_Physics'
existing = [a for a in subsystem.get_all_level_actors() if a.get_actor_label() == label]
actor = existing[0] if existing else subsystem.spawn_actor_from_class(
    unreal.load_class(None, '/Script/RapyutaSimulationPlugins.TandemTurtleBot'),
    unreal.Vector(9000, 700, 3), unreal.Rotator(0, 0, 0))
assert actor
actor.set_actor_label(label)
actor.set_folder_path('Fable_Oval_Preview/TandemTurtleBot')
report = {'actor': actor.get_path_name(), 'bodies': []}
for child in actor.get_components_by_class(unreal.ChildActorComponent):
    bot = child.get_editor_property('child_actor')
    assert bot, 'Original Burger Blueprint failed to load'
    for body in bot.get_components_by_class(unreal.StaticMeshComponent):
        bi = body.get_editor_property('body_instance')
        row = {'module': child.get_name(), 'component': body.get_name(),
               'mesh': body.static_mesh.get_path_name() if body.static_mesh else None}
        assert row['mesh'], f'Missing original mesh: {row}'
        for field in ('mass_in_kg_override', 'override_mass', 'linear_damping', 'angular_damping',
                      'inertia_tensor_scale', 'com_nudge', 'phys_material_override', 'collision_enabled'):
            try:
                value = bi.get_editor_property(field)
                row[field] = value if isinstance(value, (str, int, float, bool)) else str(value)
            except Exception as exc:
                row[field] = str(exc)
        report['bodies'].append(row)
assert len(report['bodies']) == 10, report
out = Path(unreal.Paths.project_dir()) / 'simulation_ml/integrations/rapyuta/validation'
out.mkdir(parents=True, exist_ok=True)
(out / 'loaded_physics.json').write_text(json.dumps(report, indent=2))
assert level.save_current_level()
unreal.log('TANDEM_SETUP_OK ' + str(out))
