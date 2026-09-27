"""Executed by the editor:  UnrealEditor.exe <project> -ExecutePythonScript="<this file>"
Builds the BoatLab course into the startup level (Lake_300x80ft) and saves it."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "Content", "Python"))
import boatlab_course  # noqa: E402

print(boatlab_course.build())
