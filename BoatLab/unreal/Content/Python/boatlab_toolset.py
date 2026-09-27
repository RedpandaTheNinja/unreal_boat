"""
boatlab_toolset.py - exposes the BoatLab course tools through Unreal 5.8's built-in MCP server
(plugins: "Model Context Protocol" + "Toolset Registry", already enabled in mcp_gpt_boat.uproject).

Register once per editor session (or add this file under Project Settings -> Plugins -> Python ->
Startup Scripts). Importing registers the toolset; re-importing (importlib.reload) re-registers it:
    import sys; sys.path.append(r"G:/My Drive/Unreal projects/mcp_gpt_boat/BoatLab/unreal/Content/Python")
    import boatlab_toolset

Any MCP client connected to http://127.0.0.1:8000/mcp (Claude, Codex) then sees the toolset
"boatlab_toolset.BoatLabCourseToolset" via list_toolsets / call_tool.
Coordinates are BoatLab ENU metres (x east, y north), headings in degrees CCW from east.
"""
from __future__ import annotations

import importlib
import json
import os
import sys

import unreal

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import boatlab_course as bc  # noqa: E402

importlib.reload(bc)  # pick up edits to the builder when the toolset is re-imported

try:
    import toolset_registry
except ImportError:
    unreal.log_warning("[boatlab] toolset_registry not importable - enable 'Model Context Protocol' and "
                       "'Toolset Registry' plugins (UE 5.8). boatlab_course functions still work from the Python console.")
    raise

# On reload, unregister before the class below is redefined: redefinition re-instances the UClass,
# and a registration made against the old one is left stale (listed with no tools, calls fail).
_previous = globals().get("_registered_class")
if _previous is not None and unreal.ToolsetRegistry.is_toolset_class_registered(_previous):
    unreal.ToolsetRegistry.unregister_toolset_class(_previous)


@unreal.uclass()
class BoatLabCourseToolset(unreal.ToolsetDefinition):
    """Build, inspect and edit the AIMM-ICC boat course and boat start pose in the open lake level
    (Lake_300x80ft). The course JSON (BoatLab/config/course_aimm_2025.json) is the source of truth
    shared with the Python autonomy stack. Coordinates are BoatLab ENU metres (x east, y north)."""

    @toolset_registry.tool_call
    @staticmethod
    def build_course(fit_lake: bool = True) -> str:
        """Places every course object and the marina (piers, moored boats) from the course JSON into the
        open level, moves the boat to the course start pose and saves the level.

        Removes previously placed BoatLab actors (and the level's original stub pier) first, so it is safe
        to run again.

        Args:
            fit_lake: Also size the water, lake bed, shore and grass to the course JSON water rectangle.

        Returns:
            JSON summary with the placed object ids, marina part count, lake size and boat start pose.
        """
        return json.dumps(bc.build(fit=fit_lake))

    @toolset_registry.tool_call
    @staticmethod
    def clear_course() -> str:
        """Deletes every actor BoatLab placed (tag "boatlab"). Hand-placed actors are kept.

        Returns:
            JSON with the number of actors removed.
        """
        return json.dumps({"removed": bc.clear()})

    @toolset_registry.tool_call
    @staticmethod
    def describe_course() -> str:
        """Lists BoatLab course actors and the boat actor with ENU positions in metres.

        Returns:
            JSON list of {label, x, y, z, tags}; the boat entry has heading_deg instead of z/tags.
        """
        return json.dumps(bc.describe())

    @toolset_registry.tool_call
    @staticmethod
    def move_course_object(object_id: str, x_m: float, y_m: float) -> str:
        """Moves one course object (e.g. "zebra", "gateA_red") in the level and in the course JSON,
        so the Python planner sees the same position.

        Args:
            object_id: Object id from the course JSON.
            x_m: New east coordinate, metres.
            y_m: New north coordinate, metres.

        Returns:
            JSON with the moved id and number of actors updated.
        """
        return json.dumps(bc.move_object(object_id, x_m, y_m))

    @toolset_registry.tool_call
    @staticmethod
    def set_boat_start(x_m: float, y_m: float, heading_deg: float) -> str:
        """Moves the boat actor to a new Play-start pose (editor only, not during Play).

        Args:
            x_m: East coordinate, metres.
            y_m: North coordinate, metres.
            heading_deg: Heading in degrees, counter-clockwise from east (180 = facing west).

        Returns:
            JSON with the applied pose.
        """
        return json.dumps(bc.set_boat_start(x_m, y_m, heading_deg))

    @toolset_registry.tool_call
    @staticmethod
    def fit_lake() -> str:
        """Sizes the water, lake bed, shore and grass to the course JSON water rectangle and saves the level.
        Idempotent. Change the lake size in tools/make_course.py (lake_x_ft, lake_y_ft) and regenerate the JSON.

        Returns:
            JSON with the lake length/width in feet and the adjusted actors.
        """
        return json.dumps(bc.fit_lake())


unreal.ToolsetRegistry.register_toolset_class(BoatLabCourseToolset)
_registered_class = BoatLabCourseToolset

if unreal.ToolsetRegistry.is_toolset_class_registered(BoatLabCourseToolset):
    unreal.log("[boatlab] BoatLabCourseToolset registered with the toolset registry")
else:
    unreal.log_warning("[boatlab] BoatLabCourseToolset registration failed")
