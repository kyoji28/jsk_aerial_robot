#!/usr/bin/env python3
"""Interactive XYZ translation commands for DRAGON (ROS 1).

Move the CoG in the WORLD frame using position + zero-velocity targets.
Existing DRAGON controller and PID configuration are not modified.

Commands:
    move dx dy dz    Relative translation from current measured CoG position [m]
    x    dx          Relative X translation from measured CoG position [m]
    y    dy          Relative Y translation from measured CoG position [m]
    z    dz          Relative Z translation from measured CoG position [m]
    set  x  y  z     Absolute world-frame CoG target [m]
    hold             Target current measured CoG position
    status           Flight state, measured position, latest commanded target
    help / quit

This tool intentionally does not arm, take off, or land the robot.
"""

import math
import time

import rospy
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8


class DragonTranslationTeleop:
    HOVER = 5
    STATES = {
        0: "ARM_OFF", 1: "START", 2: "ARM_ON", 3: "TAKEOFF",
        4: "LAND", 5: "HOVER", 6: "STOP",
    }

    def __init__(self):
        self.ns = rospy.get_param("~robot_ns", "/dragon").rstrip("/")
        self.max_move = float(rospy.get_param("~max_move", 0.5))
        self.min_z = float(rospy.get_param("~min_z", 0.4))
        self.max_data_age = float(rospy.get_param("~max_data_age", 2.0))

        if not math.isfinite(self.max_move) or self.max_move <= 0:
            raise ValueError("~max_move must be positive and finite")
        if not math.isfinite(self.min_z) or self.min_z < 0:
            raise ValueError("~min_z must be nonnegative and finite")
        if not math.isfinite(self.max_data_age) or self.max_data_age <= 0:
            raise ValueError("~max_data_age must be positive and finite")

        self.position = None
        self.odom_received = None
        self.state = None
        self.state_received = None
        self.last_target = None

        self.nav_pub = rospy.Publisher(
            self.ns + "/uav/nav", FlightNav, queue_size=10
        )
        rospy.Subscriber(
            self.ns + "/uav/cog/odom", Odometry, self.on_odom, queue_size=1
        )
        rospy.Subscriber(
            self.ns + "/flight_state", UInt8, self.on_state, queue_size=1
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(v) for v in values):
            rospy.logwarn_throttle(2.0, "Ignoring invalid CoG position")
            return
        self.position = values
        self.odom_received = time.monotonic()

    def on_state(self, msg):
        self.state = msg.data
        self.state_received = time.monotonic()

    def is_fresh(self, received):
        return (received is not None and
                time.monotonic() - received <= self.max_data_age)

    def status(self):
        name = self.STATES.get(self.state, "UNKNOWN")
        print("Flight state: {} ({})".format(self.state, name))
        if not self.is_fresh(self.state_received):
            print("  Warning: flight state missing or stale")
        if self.position is None:
            print("CoG position: not received yet")
        else:
            print("CoG position (world): x={:.3f}, y={:.3f}, z={:.3f} m".format(
                *self.position))
            if not self.is_fresh(self.odom_received):
                print("  Warning: odometry missing or stale")
        if self.last_target is not None:
            print("Last target (world): x={:.3f}, y={:.3f}, z={:.3f} m".format(
                *self.last_target))

    def send_position(self, xyz):
        """Send an absolute CoG position target in the world frame."""
        if not all(math.isfinite(v) for v in xyz):
            print("Target position must contain finite numbers.")
            return
        if self.state != self.HOVER or not self.is_fresh(self.state_received):
            print("Movement requires HOVER (flight_state=5) with a recent state message.")
            return
        if self.position is None or not self.is_fresh(self.odom_received):
            print("No recent odometry. Check {}/uav/cog/odom.".format(self.ns))
            return
        if self.nav_pub.get_num_connections() == 0:
            print("No subscriber on {}/uav/nav. Check the control node.".format(self.ns))
            return
        if xyz[2] < self.min_z:
            print("Refusing target z={:.3f} m below min_z={:.3f} m.".format(
                xyz[2], self.min_z))
            return

        nav = FlightNav()
        nav.header.stamp = rospy.Time.now()
        nav.control_frame = FlightNav.WORLD_FRAME
        nav.target = FlightNav.COG
        nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
        nav.target_pos_x, nav.target_pos_y, nav.target_pos_z = xyz
        nav.target_vel_x = 0.0
        nav.target_vel_y = 0.0
        nav.target_vel_z = 0.0
        # yaw_nav_mode is NO_NAVIGATION (existing yaw target unchanged).
        self.nav_pub.publish(nav)
        self.last_target = tuple(xyz)
        print("Sent world CoG target: x={:.3f}, y={:.3f}, z={:.3f} m".format(*xyz))
        print("  Wait for the robot to settle before the next command.")

    def move_relative(self, dx, dy, dz):
        """Move relative to the current measured world-frame CoG position."""
        delta = (dx, dy, dz)
        if not all(math.isfinite(v) for v in delta):
            print("Displacement must contain finite numbers.")
            return
        if math.sqrt(sum(v * v for v in delta)) > self.max_move + 1e-9:
            print("One relative move is limited to {:.2f} m. Change ~max_move if needed.".format(
                self.max_move))
            return
        if self.position is None or not self.is_fresh(self.odom_received):
            print("No recent odometry to compute a relative target.")
            return
        target = tuple(p + d for p, d in zip(self.position, delta))
        self.send_position(target)

    def help(self):
        print("""
========= DRAGON Translation Teleop =========
  move dx dy dz    Relative move from measured CoG position [m]
  x    dx          Relative X increment from measured CoG position [m]
  y    dy          Relative Y increment from measured CoG position [m]
  z    dz          Relative Z increment from measured CoG position [m]
  set  x  y  z     Absolute world-frame CoG target [m]
  hold             Set target to current measured position
  status           Show flight state, position and last target
  help             Show commands
  quit (q)         Exit only (does NOT land)
=============================================
Movement commands are accepted only while HOVERING.
'relative move' is limited to {:.2f} m per command.
Minimum target altitude: {:.2f} m.
Wait for the robot to settle before the next command.
""".format(self.max_move, self.min_z))

    def run(self):
        self.help()
        while not rospy.is_shutdown():
            try:
                args = input("xyz> ").strip().lower().split()
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
                elif cmd == "move" and len(args) == 4:
                    self.move_relative(*map(float, args[1:]))
                elif cmd in ("x", "y", "z") and len(args) == 2:
                    value = float(args[1])
                    deltas = {
                        "x": (value, 0.0, 0.0),
                        "y": (0.0, value, 0.0),
                        "z": (0.0, 0.0, value),
                    }
                    self.move_relative(*deltas[cmd])
                elif cmd == "set" and len(args) == 4:
                    self.send_position(tuple(map(float, args[1:])))
                elif cmd == "hold" and len(args) == 1:
                    if self.position is None or not self.is_fresh(self.odom_received):
                        print("No recent odometry to hold the current position.")
                    else:
                        self.send_position(self.position)
                else:
                    print("Unknown command or arguments. Type 'help'.")
            except ValueError:
                print("Invalid number. Example: x 0.2, move 0.3 0 0, or set 0 0 1.0")


if __name__ == "__main__":
    rospy.init_node("dragon_translation_teleop")
    try:
        DragonTranslationTeleop().run()
    finally:
        rospy.signal_shutdown("Translation teleop stopped")
