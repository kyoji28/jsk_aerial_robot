#!/usr/bin/env python3
"""Interactive translation commands for DRAGON on ROS 1 (Gazebo first).

Works alongside pid_gain_tuner.py; does not modify the original controller.
All translations use the WORLD frame and the robot CoG.
"""

import math

import rospy
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8


class DragonTranslationTeleop:
    HOVER = 5
    STATES = {
        0: "ARM_OFF", 1: "START", 2: "ARM_ON", 3: "TAKEOFF",
        4: "LAND", 5: "HOVER", 6: "STOP",
    }

    def __init__(self):
        ns = rospy.get_param("~robot_ns", "/dragon").rstrip("/")
        self.step = float(rospy.get_param("~step", 0.2))
        self.max_move = float(rospy.get_param("~max_move", 0.5))
        self.min_z = float(rospy.get_param("~min_z", 0.4))
        self.position = None
        self.state = None
        self.last_target = None

        self.nav_pub = rospy.Publisher(ns + "/uav/nav", FlightNav, queue_size=10)
        self.arm_pub = rospy.Publisher(ns + "/teleop_command/start", Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(ns + "/teleop_command/takeoff", Empty, queue_size=1)
        self.land_pub = rospy.Publisher(ns + "/teleop_command/land", Empty, queue_size=1)
        rospy.Subscriber(ns + "/uav/cog/odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(ns + "/flight_state", UInt8, self.on_state, queue_size=1)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y, p.z)

    def on_state(self, msg):
        self.state = msg.data

    def status(self):
        name = self.STATES.get(self.state, "UNKNOWN")
        print("Flight state: {} ({})".format(self.state, name))
        if self.position is None:
            print("Position: not received yet")
        else:
            print("CoG position (world): x={:.3f}, y={:.3f}, z={:.3f} m".format(*self.position))
        if self.last_target is not None:
            print("Last target (world): x={:.3f}, y={:.3f}, z={:.3f} m".format(*self.last_target))

    def send_flight_command(self, kind):
        pub, expected = {
            "arm": (self.arm_pub, 0),
            "takeoff": (self.takeoff_pub, 2),
            "land": (self.land_pub, 5),
        }[kind]
        if self.state != expected:
            print("Cannot {}: expected state {}, current {}. Run 'status'.".format(
                kind, expected, self.state))
            return
        if pub.get_num_connections() == 0:
            print("No subscriber for '{}'; check the DRAGON launch.".format(kind))
            return
        pub.publish(Empty())
        print("Sent {} command. Use 'status' to check the transition.".format(kind))

    def send_position(self, xyz):
        if self.state != self.HOVER:
            print("Movement is allowed only in HOVER (flight_state=5).")
            return
        if self.position is None:
            print("No odometry yet. Check /dragon/uav/cog/odom.")
            return
        if self.nav_pub.get_num_connections() == 0:
            print("No subscriber on /uav/nav. Check the control node.")
            return
        if not all(math.isfinite(x) for x in xyz):
            print("Position must contain finite numbers.")
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
        # yaw_nav_mode remains NO_NAVIGATION (preserve current yaw target).
        self.nav_pub.publish(nav)
        self.last_target = tuple(xyz)
        print("Sent target: x={:.3f}, y={:.3f}, z={:.3f} m".format(*xyz))

    def move_relative(self, dx, dy, dz):
        if self.position is None:
            print("No odometry available yet.")
            return
        delta = (dx, dy, dz)
        if not all(math.isfinite(v) for v in delta):
            print("Displacement must contain finite numbers.")
            return
        if math.sqrt(sum(v * v for v in delta)) > self.max_move + 1e-9:
            print("One command is limited to {:.2f} m. Change ~max_move if needed.".format(
                self.max_move))
            return
        # Relative to current measured position, not previous target.
        target = tuple(p + d for p, d in zip(self.position, delta))
        self.send_position(target)

    def help(self):
        print("""
========= DRAGON Translation Teleop =========
  arm (r)          Arm motors
  takeoff (t)      Take off
  land (l)         Land
  status           Show flight state and CoG position

  w / s            +X / -X by one step (world frame)
  a / d            +Y / -Y by one step (world frame)
  u / j            +Z / -Z by one step (world frame)
  move dx dy dz    Relative translation in meters
  step value       Change the step distance in meters
  hold             Set target to current measured position

  help             Show commands
  quit (q)         Quit this tool (does NOT land)
===========================================
Wait until the robot has settled before sending the next move.
""")

    def run(self):
        self.help()
        while not rospy.is_shutdown():
            try:
                args = input("dragon> ").strip().lower().split()
            except (KeyboardInterrupt, EOFError):
                print()
                break
            if not args:
                continue
            cmd = args[0]
            try:
                if cmd in ("arm", "r", "takeoff", "t", "land", "l"):
                    if len(args) != 1:
                        print("Usage: {}".format(cmd))
                        continue
                    command = {"r": "arm", "t": "takeoff", "l": "land"}.get(cmd, cmd)
                    self.send_flight_command(command)
                elif cmd == "status":
                    self.status()
                elif cmd == "help":
                    self.help()
                elif cmd in ("q", "quit", "exit"):
                    if self.state == self.HOVER:
                        print("Warning: robot is still in HOVER. Quitting does not land it.")
                    break
                elif cmd == "step" and len(args) == 2:
                    value = float(args[1])
                    if not (math.isfinite(value) and 0.0 < value <= self.max_move):
                        print("Step must be > 0 and <= {:.2f} m.".format(self.max_move))
                        continue
                    self.step = value
                    print("Step set to {:.3f} m".format(self.step))
                elif cmd == "move" and len(args) == 4:
                    self.move_relative(*map(float, args[1:]))
                elif cmd == "hold" and len(args) == 1:
                    if self.position is None:
                        print("No odometry available yet.")
                    else:
                        self.send_position(self.position)
                elif cmd in ("w", "s", "a", "d", "u", "j") and len(args) == 1:
                    unit = {"w": (1, 0, 0), "s": (-1, 0, 0),
                            "a": (0, 1, 0), "d": (0, -1, 0),
                            "u": (0, 0, 1), "j": (0, 0, -1)}[cmd]
                    self.move_relative(*(v * self.step for v in unit))
                else:
                    print("Unknown command or arguments. Type 'help'.")
            except ValueError:
                print("Invalid number. Example: move 0.3 0 0")


if __name__ == "__main__":
    rospy.init_node("dragon_translation_teleop")
    try:
        DragonTranslationTeleop().run()
    finally:
        rospy.signal_shutdown("Translation teleop stopped")
