#!/usr/bin/env python3
"""Interactive DRAGON CoG-attitude teleoperation for ROS 1 (Gazebo first).

Publishes nav_msgs/Odometry to /dragon/target_rotation_motion with
header.frame_id='cog'. DRAGON's DragonNavigator maps this message to
setTargetRPY / setTargetOmega, so Roll, Pitch and Yaw are handled by the
existing attitude PID controllers. No changes to C++ or PID YAML are needed.

Angles entered in the console are in DEGREES, interpreted as Euler RPY
in the WORLD reference. Relative commands add Euler-angle increments to
CURRENT MEASURED CoG attitude (not to the previous target). They are NOT
body-fixed-axis incremental rotations.
"""

import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def wrap_deg(angle):
    """Normalize yaw to [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


class DragonRotationTeleop:
    HOVER = 5
    STATES = {
        0: "ARM_OFF", 1: "START", 2: "ARM_ON", 3: "TAKEOFF",
        4: "LAND", 5: "HOVER", 6: "STOP",
    }

    def __init__(self):
        ns = rospy.get_param("~robot_ns", "/dragon").rstrip("/")
        self.step_deg = float(rospy.get_param("~step_deg", 5.0))
        self.max_tilt_deg = float(rospy.get_param("~max_tilt_deg", 20.0))
        self.max_change_tilt_deg = float(rospy.get_param("~max_change_tilt_deg", 20.0))
        self.max_change_yaw_deg = float(rospy.get_param("~max_change_yaw_deg", 30.0))
        self.min_height = float(rospy.get_param("~min_height", 0.5))
        self.max_data_age = float(rospy.get_param("~max_data_age", 2.0))

        if not all(math.isfinite(v) and v > 0 for v in (
                self.step_deg, self.max_tilt_deg,
                self.max_change_tilt_deg, self.max_change_yaw_deg,
                self.max_data_age)):
            raise ValueError("step/angle limits/max_data_age must be finite and positive")
        if not math.isfinite(self.min_height) or self.min_height < 0:
            raise ValueError("min_height must be finite and nonnegative")
        if self.step_deg > min(self.max_change_tilt_deg, self.max_change_yaw_deg):
            raise ValueError("step_deg exceeds a per-command angle change limit")

        self.state = None
        self.state_received = None
        self.rpy_deg = None
        self.z = None
        self.odom_received = None
        self.last_target = None

        self.rot_pub = rospy.Publisher(
            ns + "/target_rotation_motion", Odometry, queue_size=1
        )
        self.arm_pub = rospy.Publisher(ns + "/teleop_command/start", Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(ns + "/teleop_command/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher(ns + "/teleop_command/land", Empty, queue_size=1)
        rospy.Subscriber(ns + "/uav/cog/odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(ns + "/flight_state", UInt8, self.on_state, queue_size=1)
        self.ns = ns

    def on_state(self, msg):
        self.state = msg.data
        self.state_received = time.monotonic()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        vals = (q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v * v for v in vals))
        if not all(math.isfinite(v) for v in vals) or norm < 1e-6:
            rospy.logwarn_throttle(2.0, "Ignoring invalid CoG orientation quaternion")
            return
        angles_rad = euler_from_quaternion(tuple(v / norm for v in vals))
        new_rpy = tuple(math.degrees(v) for v in angles_rad)
        z = msg.pose.pose.position.z
        if not all(math.isfinite(v) for v in new_rpy) or not math.isfinite(z):
            return
        self.rpy_deg = new_rpy
        self.z = z
        self.odom_received = time.monotonic()

    def is_fresh(self, received):
        return received is not None and time.monotonic() - received <= self.max_data_age

    def status(self):
        print("Flight state: {} ({})".format(
            self.state, self.STATES.get(self.state, "UNKNOWN")))
        if not self.is_fresh(self.state_received):
            print("  Warning: flight state missing or stale")
        if self.rpy_deg is None:
            print("CoG attitude: not received yet")
        else:
            print("CoG measured: Roll={:+.2f}, Pitch={:+.2f}, Yaw={:+.2f} deg; z={:.2f} m".format(
                *self.rpy_deg, self.z))
            if not self.is_fresh(self.odom_received):
                print("  Warning: odometry stale")
        if self.last_target is not None:
            print("Last commanded: Roll={:+.2f}, Pitch={:+.2f}, Yaw={:+.2f} deg".format(
                *self.last_target))

    def flight_command(self, kind):
        pub, expected = {
            "arm": (self.arm_pub, 0),
            "takeoff": (self.takeoff_pub, 2),
            "land": (self.land_pub, 5),
        }[kind]
        if not self.is_fresh(self.state_received):
            print("Cannot {}: flight_state missing/stale".format(kind))
            return
        if self.state != expected:
            print("Cannot {}: expected state {}, current {}. Run 'status'.".format(
                kind, expected, self.state))
            return
        if pub.get_num_connections() == 0:
            print("No subscriber for '{}'. Check bringup.launch.".format(kind))
            return
        pub.publish(Empty())
        print("Sent {}. Run 'status' to confirm the transition.".format(kind))

    def send_target(self, roll, pitch, yaw):
        if self.state != self.HOVER or not self.is_fresh(self.state_received):
            print("Attitude commands require HOVER (flight_state=5).")
            return
        if not self.is_fresh(self.odom_received) or self.rpy_deg is None:
            print("No recent CoG odometry. Check {}/uav/cog/odom.".format(self.ns))
            return
        if self.z < self.min_height:
            print("Refusing attitude command below {:.2f} m altitude.".format(self.min_height))
            return
        if self.rot_pub.get_num_connections() == 0:
            print("No subscriber on {}/target_rotation_motion.".format(self.ns))
            return
        if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
            print("Target angles must be finite numbers.")
            return

        if abs(roll) > self.max_tilt_deg or abs(pitch) > self.max_tilt_deg:
            print("Roll/Pitch targets are limited to +/-{:.1f} deg.".format(self.max_tilt_deg))
            return

        current_roll, current_pitch, current_yaw = self.rpy_deg
        if (abs(roll - current_roll) > self.max_change_tilt_deg + 1e-8 or
                abs(pitch - current_pitch) > self.max_change_tilt_deg + 1e-8 or
                abs(wrap_deg(yaw - current_yaw)) > self.max_change_yaw_deg + 1e-8):
            print("Change too large: max Roll/Pitch={:.1f} deg, Yaw={:.1f} deg per command.".format(
                self.max_change_tilt_deg, self.max_change_yaw_deg))
            return

        yaw = wrap_deg(yaw)
        quat = quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw))
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "cog"  # Exact value required by DragonNavigator
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        # target angular velocities default to zero
        self.rot_pub.publish(msg)
        self.last_target = (roll, pitch, yaw)
        print("Sent CoG RPY target: R={:+.1f}, P={:+.1f}, Y={:+.1f} deg".format(
            roll, pitch, yaw))
        print("  Wait for the robot to settle before the next command.")

    def rotate_relative(self, dr, dp, dy):
        if not all(math.isfinite(v) for v in (dr, dp, dy)):
            print("Angles must be finite numbers.")
            return
        if self.rpy_deg is None or not self.is_fresh(self.odom_received):
            print("No recent CoG attitude measurement.")
            return
        r, p, y = self.rpy_deg
        self.send_target(r + dr, p + dp, y + dy)

    def help(self):
        print("""
========== DRAGON Rotation Teleop ==========
  arm / takeoff / land   Flight commands
  status                 Measured CoG RPY & flight state

  roll 5                 +5 deg Roll from current measured angle
  pitch -5               -5 deg Pitch from current measured angle
  yaw 15                 +15 deg Yaw from current measured angle
  rotate 5 0 10          Relative RPY increments (deg)
  set 0 0 0              Absolute world-referenced RPY target (deg)

  r+ / r-                +/- one-step Roll
  p+ / p-                +/- one-step Pitch
  y+ / y-                +/- one-step Yaw
  step 3                 Set step angle to 3 deg
  level                  Set Roll/Pitch to 0, keep measured Yaw
  hold                   Hold current measured RPY
  help                   Display this help
  quit                   Exit only (does not land)
============================================
RPY commands are for CoG attitude in the world frame.
Relative commands add Euler angles, not body-frame rotations.
Wait until hovering and settled before testing new PID gains.
""")

    def run(self):
        self.help()
        while not rospy.is_shutdown():
            try:
                args = input("rpy> ").strip().lower().split()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not args:
                continue
            cmd = args[0]
            try:
                if cmd in ("arm", "takeoff", "land") and len(args) == 1:
                    self.flight_command(cmd)
                elif cmd == "status" and len(args) == 1:
                    self.status()
                elif cmd == "help" and len(args) == 1:
                    self.help()
                elif cmd in ("q", "quit", "exit") and len(args) == 1:
                    if self.state == self.HOVER:
                        print("WARNING: drone may still be flying. Quit does not land.")
                    break
                elif cmd == "step" and len(args) == 2:
                    val = float(args[1])
                    if not (math.isfinite(val) and 0 < val <= min(
                            self.max_change_tilt_deg, self.max_change_yaw_deg)):
                        print("Step must be positive and within the angle change limits.")
                    else:
                        self.step_deg = val
                        print("Step set to {:.1f} deg".format(val))
                elif cmd in ("roll", "pitch", "yaw") and len(args) == 2:
                    value = float(args[1])
                    parts = {"roll": (value, 0.0, 0.0),
                             "pitch": (0.0, value, 0.0),
                             "yaw": (0.0, 0.0, value)}
                    self.rotate_relative(*parts[cmd])
                elif cmd == "rotate" and len(args) == 4:
                    self.rotate_relative(*map(float, args[1:]))
                elif cmd == "set" and len(args) == 4:
                    self.send_target(*map(float, args[1:]))
                elif cmd in ("r+", "r-", "p+", "p-", "y+", "y-") and len(args) == 1:
                    axis, sign = cmd[0], (1 if cmd[1] == "+" else -1)
                    delta = sign * self.step_deg
                    self.rotate_relative(*{
                        "r": (delta, 0.0, 0.0),
                        "p": (0.0, delta, 0.0),
                        "y": (0.0, 0.0, delta),
                    }[axis])
                elif cmd in ("level", "hold") and len(args) == 1:
                    if self.rpy_deg is None or not self.is_fresh(self.odom_received):
                        print("No recent CoG attitude measurement.")
                    elif cmd == "hold":
                        self.send_target(*self.rpy_deg)
                    else:
                        self.send_target(0.0, 0.0, self.rpy_deg[2])
                else:
                    print("Unknown command/arguments. Type 'help'.")
            except ValueError:
                print("Invalid input. Example: rotate 5 0 10")


if __name__ == "__main__":
    rospy.init_node("dragon_rotation_teleop")
    try:
        DragonRotationTeleop().run()
    finally:
        rospy.signal_shutdown("Rotation teleop stopped")
