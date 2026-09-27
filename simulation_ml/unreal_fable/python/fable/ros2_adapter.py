"""
ros2_adapter.py - run the SAME controller on the real vehicle.

The controller never sees ROS. This node builds a fable `Observation` from ROS 2
topics, calls controller(obs), and publishes the command. Because the wire
convention of the bridge is already REP-103 (ENU, body x-forward y-left, SI),
nothing is converted here except message types.

Topics (remap to taste):
  in:   /odom           nav_msgs/Odometry        pose + body twist
        /imu            sensor_msgs/Imu          angular rate, accel
        /scan           sensor_msgs/LaserScan    2-D lidar (optional)
        /gps/fix        sensor_msgs/NavSatFix    (optional)
        /bumper         std_msgs/Bool            contact (optional)
  out:  /cmd_ackermann  ackermann_msgs/AckermannDriveStamped   (car)
        /cmd_thrust     std_msgs/Float32MultiArray [l, r]      (boat)
        /cmd_vel        geometry_msgs/Twist                    (car, alternative)

Runs on Linux (Jetson/Pi on the vehicle) with ROS 2 Humble/Jazzy. On the
Windows dev box you run the sim; this file is for the robot.

    ros2 run  ->  python3 -m fable.ros2_adapter --scene specs/worlds/car_track_oval.json \
                       --vehicle specs/vehicles/car_rc_measured.json --controller pure_pursuit
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

from .specs import load_scene, load_vehicle
from .types import Lidar, Observation, Pose, Twist


def _yaw_from_quat(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_controller(name: str, scene, vehicle):
    from .controllers import DockingController, LOSGuidance, PurePursuit, ReactiveAvoid
    table = {
        "pure_pursuit": lambda: PurePursuit(scene, vehicle),
        "pure_pursuit_avoid": lambda: ReactiveAvoid(PurePursuit(scene, vehicle), "car"),
        "los": lambda: LOSGuidance(scene, vehicle),
        "los_avoid": lambda: ReactiveAvoid(LOSGuidance(scene, vehicle), "boat"),
        "dock": lambda: DockingController(scene, vehicle),
    }
    return table[name]()


class FableRos2Node:
    def __init__(self, scene_path: str, vehicle_path: str | None, controller: str, rate_hz: float | None = None):
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan, NavSatFix
        from std_msgs.msg import Bool, Float32MultiArray

        self.scene = load_scene(scene_path)
        self.vehicle = load_vehicle(vehicle_path) if vehicle_path else self.scene.vehicle()
        self.ctrl = make_controller(controller, self.scene, self.vehicle)
        self.dt = 1.0 / (rate_hz or float(self.vehicle.common.get("control_rate_hz", 50)))

        rclpy.init()
        self.node = Node("fable_controller")
        self.obs = Observation(vehicle="car" if self.vehicle.is_car else "boat", dt=self.dt)
        self.t0 = time.monotonic()
        self.node.create_subscription(Odometry, "/odom", self._odom, 10)
        self.node.create_subscription(Imu, "/imu", self._imu, 10)
        self.node.create_subscription(LaserScan, "/scan", self._scan, 5)
        self.node.create_subscription(NavSatFix, "/gps/fix", self._gps, 5)
        self.node.create_subscription(Bool, "/bumper", self._bumper, 5)

        if self.vehicle.is_car:
            try:
                from ackermann_msgs.msg import AckermannDriveStamped
                self.pub = self.node.create_publisher(AckermannDriveStamped, "/cmd_ackermann", 10)
                self._publish = self._pub_ackermann
            except ImportError:
                from geometry_msgs.msg import Twist as TwistMsg
                self.pub = self.node.create_publisher(TwistMsg, "/cmd_vel", 10)
                self._publish = self._pub_twist
        else:
            self.pub = self.node.create_publisher(Float32MultiArray, "/cmd_thrust", 10)
            self._publish = self._pub_thrust

        self.node.create_timer(self.dt, self._tick)
        self.tick = 0
        self.rclpy = rclpy

    # --- subscriptions -> Observation
    def _odom(self, m):
        p, t = m.pose.pose, m.twist.twist
        self.obs.pose = Pose(p.position.x, p.position.y, p.position.z, yaw=_yaw_from_quat(p.orientation))
        self.obs.vel.vx, self.obs.vel.vy = t.linear.x, t.linear.y
        self.obs.speed = math.hypot(t.linear.x, t.linear.y)

    def _imu(self, m):
        self.obs.vel.wz = m.angular_velocity.z
        self.obs.acc = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)

    def _scan(self, m):
        r = np.asarray(m.ranges, dtype=np.float32)
        r[~np.isfinite(r)] = m.range_max
        self.obs.lidar = Lidar(m.angle_min, m.angle_max, m.range_max, np.clip(r, 0, m.range_max))

    def _gps(self, m):
        self.obs.gps = {"lat": m.latitude, "lon": m.longitude, "hdop": 1.0}

    def _bumper(self, m):
        self.obs.collision = bool(m.data)

    # --- command publishers
    def _pub_ackermann(self, u):
        from ackermann_msgs.msg import AckermannDriveStamped
        msg = AckermannDriveStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        c = self.vehicle.car
        msg.drive.steering_angle = float(u.get("steer", 0.0)) * float(c["max_steer_rad"])
        msg.drive.speed = float(u.get("throttle", 0.0)) * float(c["max_speed_mps"]) * (1.0 - float(u.get("brake", 0.0)))
        self.pub.publish(msg)

    def _pub_twist(self, u):
        from geometry_msgs.msg import Twist as TwistMsg
        c = self.vehicle.car
        msg = TwistMsg()
        v = float(u.get("throttle", 0.0)) * float(c["max_speed_mps"]) * (1.0 - float(u.get("brake", 0.0)))
        msg.linear.x = v
        msg.angular.z = v * math.tan(float(u.get("steer", 0.0)) * float(c["max_steer_rad"])) / float(c["wheelbase_m"])
        self.pub.publish(msg)

    def _pub_thrust(self, u):
        from std_msgs.msg import Float32MultiArray
        msg = Float32MultiArray()
        msg.data = [float(u.get("thrust_l", 0.0)), float(u.get("thrust_r", 0.0))]
        self.pub.publish(msg)

    # --- control loop
    def _tick(self):
        self.tick += 1
        self.obs.tick = self.tick
        self.obs.t = time.monotonic() - self.t0
        u = self.ctrl(self.obs)
        self._publish(u)

    def spin(self):
        try:
            self.rclpy.spin(self.node)
        finally:
            self.node.destroy_node()
            self.rclpy.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--vehicle")
    ap.add_argument("--controller", default="pure_pursuit",
                    choices=["pure_pursuit", "pure_pursuit_avoid", "los", "los_avoid", "dock"])
    ap.add_argument("--rate", type=float)
    a = ap.parse_args()
    try:
        import rclpy  # noqa: F401
    except ImportError:
        sys.exit("rclpy not found: source your ROS 2 install (this file runs on the vehicle, not the sim PC)")
    FableRos2Node(a.scene, a.vehicle, a.controller, a.rate).spin()


if __name__ == "__main__":
    main()
