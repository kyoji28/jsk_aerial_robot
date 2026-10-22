#!/usr/bin/env python3
"""Interactive RPY rotation commands for DRAGON (ROS 1).

Set the CoG attitude in the WORLD reference using nav_msgs/Odometry on
/dragon/target_rotation_motion (header.frame_id='cog'). The existing DRAGON
attitude PID controller is used; its gains and C++ code are not modified.

Commands:
    rotate dr dp dy  Relative RPY Euler-angle increments from measured CoG [deg]
    set    r  p  y  Absolute world-referenced RPY target [deg]
    roll   dr       Relative Roll increment [deg]
    pitch  dp       Relative Pitch increment [deg]
    yaw    dy       Relative Yaw increment [deg]
    hold             Target the current measured CoG attitude
    level            Target zero Roll/Pitch, keeping measured Yaw
    status           Flight state, measured attitude, latest commanded target
    help / quit

Relative commands add world-referenced Euler-angle increments; they do NOT
perform intrinsic (body-axis) incremental rotations.

This tool intentionally does not arm, take off, or land the robot.
"""

import math
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
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
        self.ns = rospy.get_param("~robot_ns", "/dragon").rstrip("/")
        self.max_data_age = float(rospy.get_param("~max_data_age", 2.0))
        if not math.isfinite(self.max_data_age) or self.max_data_age <= 0:
            raise ValueError("~max_data_age must be positive and finite")

        self.state = None
        self.state_received = None
        self.rpy_deg = None
        self.z = None
        self.odom_received = None
        self.last_target = None

        self.rot_pub = rospy.Publisher(
            self.ns + "/target_rotation_motion", Odometry, queue_size=1
        )
        rospy.Subscriber(
            self.ns + "/uav/cog/odom", Odometry, self.on_odom, queue_size=1
        )
        rospy.Subscriber(
            self.ns + "/flight_state", UInt8, self.on_state, queue_size=1
        )

    def on_state(self, msg):
        self.state = msg.data
        self.state_received = time.monotonic()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        vals = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in vals):
            rospy.logwarn_throttle(2.0, "Ignoring invalid CoG orientation quaternion")
            return
        norm = math.sqrt(sum(v * v for v in vals))
        if norm < 1e-6:
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
        return (received is not None and
                time.monotonic() - received <= self.max_data_age)

    def status(self):
        name = self.STATES.get(self.state, "UNKNOWN")
        print("Flight state: {} ({})".format(self.state, name))
        if not self.is_fresh(self.state_received):
            print("  Warning: flight state missing or stale")
        if self.rpy_deg is None:
            print("CoG attitude: not received yet")
        else:
            print("CoG attitude (world): roll={:+.2f}, pitch={:+.2f}, yaw={:+.2f} deg; z={:.2f} m".format(
                *self.rpy_deg, self.z))
            if not self.is_fresh(self.odom_received):
                print("  Warning: odometry missing or stale")
        if self.last_target is not None:
            print("Last target (world): roll={:+.2f}, pitch={:+.2f}, yaw={:+.2f} deg".format(
                *self.last_target))

    def send_target(self, roll, pitch, yaw):
        """Send an absolute world-referenced CoG RPY target (degrees)."""
        if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
            print("Target attitude must contain finite numbers.")
            return
        if self.state != self.HOVER or not self.is_fresh(self.state_received):
            print("Rotation requires HOVER (flight_state=5) with a recent state message.")
            return
        if self.rpy_deg is None or not self.is_fresh(self.odom_received):
            print("No recent odometry. Check {}/uav/cog/odom.".format(self.ns))
            return
        if self.rot_pub.get_num_connections() == 0:
            print("No subscriber on {}/target_rotation_motion. Check the control node.".format(
                self.ns))
            return
        yaw = wrap_deg(yaw)
        quat = quaternion_from_euler(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "cog"  # Exact value required by DragonNavigator
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        # Target angular velocities default to zero.
        self.rot_pub.publish(msg)
        self.last_target = (roll, pitch, yaw)
        print("Sent world CoG RPY target: roll={:+.2f}, pitch={:+.2f}, yaw={:+.2f} deg".format(
            roll, pitch, yaw))
        print("  Wait for the robot to settle before the next command.")

    def rotate_relative(self, dr, dp, dy):
        """Add Euler-angle increments to current measured world CoG RPY."""
        delta = (dr, dp, dy)
        if not all(math.isfinite(v) for v in delta):
            print("Rotation increments must contain finite numbers.")
            return
        if self.rpy_deg is None or not self.is_fresh(self.odom_received):
            print("No recent odometry to compute a relative target.")
            return
        target = tuple(p + d for p, d in zip(self.rpy_deg, delta))
        self.send_target(*target)

    def help(self):
        print("""
========== DRAGON Rotation Teleop ==========
  rotate dr dp dy  Relative RPY rotation from measured CoG attitude [deg]
  set    r  p  y   Absolute world-referenced CoG RPY target [deg]
  roll   dr        Relative Roll increment [deg]
  pitch  dp        Relative Pitch increment [deg]
  yaw    dy        Relative Yaw increment [deg]
  hold             Set target to current measured attitude
  level            Set Roll/Pitch to 0, keep measured Yaw
  status           Show flight state, attitude and last target
  help             Show commands
  quit (q)         Exit only (does NOT land)
============================================
Rotation commands are accepted only while HOVERING.
No additional angle or altitude limits are imposed by this tool.
Relative inputs add Euler angles, not body-fixed rotations.
Wait for the robot to settle before the next command.
""")

    def run(self):
        self.help()
        while not rospy.is_shutdown():
            try:
                args = input("rpy> ").strip().lower().split()
            except (KeyboardInterrupt, EOFError):
                print()
                break
            if not args:
                continue
            cmd = args[0]
            try:
                if cmd == "status" and len(args) == 1:
                    self.status()
                elif cmd == "help" and len(args) == 1:
                    self.help()
                elif cmd in ("q", "quit", "exit") and len(args) == 1:
                    if self.state == self.HOVER:
                        print("Warning: robot may still be flying; quitting does not land it.")
                    break
                elif cmd == "rotate" and len(args) == 4:
                    self.rotate_relative(*map(float, args[1:]))
                elif cmd == "set" and len(args) == 4:
                    self.send_target(*map(float, args[1:]))
                elif cmd in ("roll", "pitch", "yaw") and len(args) == 2:
                    value = float(args[1])
                    parts = {
                        "roll": (value, 0.0, 0.0),
                        "pitch": (0.0, value, 0.0),
                        "yaw": (0.0, 0.0, value),
                    }
                    self.rotate_relative(*parts[cmd])
                elif cmd == "hold" and len(args) == 1:
                    if self.rpy_deg is None or not self.is_fresh(self.odom_received):
                        print("No recent odometry to hold the current attitude.")
                    else:
                        self.send_target(*self.rpy_deg)
                elif cmd == "level" and len(args) == 1:
                    if self.rpy_deg is None or not self.is_fresh(self.odom_received):
                        print("No recent odometry to level the current attitude.")
                    else:
                        self.send_target(0.0, 0.0, self.rpy_deg[2])
                else:
                    print("Unknown command or arguments. Type 'help'.")
            except ValueError:
                print("Invalid number. Example: rotate 5 0 10 or set 0 0 0")


if __name__ == "__main__":
    rospy.init_node("dragon_rotation_teleop")
    try:
        DragonRotationTeleop().run()
    finally:
        rospy.signal_shutdown("Rotation teleop stopped")
