"""boatnav - portable autonomy stack for the AIMM-ICC twin-motor boat.

Nothing in this package imports Unreal. The same code runs against:
  * hal.twin.TwinBoat       - Python twin of the Unreal BoatPhysics model (tests, no Unreal needed)
  * hal.unreal.UnrealBoat   - Unreal Engine 5.8 over the BoatPhysics UDP interface (port 7450)
  * hal.real.RealBoat       - Jetson + Arduino serial + GPS/MAVLink + camera (real boat)

Frames: local ENU metres (x east, y north), heading CCW from east in radians.
Body frame FLU (x forward, y left/port, z up). Motor commands in [-1, 1],
left = port motor, right = starboard motor.
"""
__version__ = "1.0.0"
