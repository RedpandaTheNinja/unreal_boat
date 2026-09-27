"""Place/inspect the tandem and leave the interactive editor open."""
import runpy
from pathlib import Path
import unreal

unreal.EditorPythonScripting.set_keep_python_script_alive(True)
runpy.run_path(str(Path(__file__).with_name('setup_tandem.py')), run_name='__main__')
