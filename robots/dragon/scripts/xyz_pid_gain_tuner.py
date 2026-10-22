#!/usr/bin/env python3
"""Interactive XY and Z PID gain tuner for the DRAGON ROS 1 controller.

The existing controller implements XY gains as one shared group and Z gains as
another group. This script changes gains at runtime using dynamic_reconfigure;
it does not modify the controller source code or its YAML settings.
"""

import math
import threading

import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, TriggerResponse


class XYZPIDGainTuner:
    GAIN_NAMES = {"p": "p_gain", "i": "i_gain", "d": "d_gain"}
    GROUPS = ("xy", "z")
    YAML_DEFAULTS = {
        "xy": {"p_gain": 3.6, "i_gain": 0.03, "d_gain": 4.0},
        "z": {"p_gain": 2.8, "i_gain": 1.2, "d_gain": 2.8},
    }

    def __init__(self):
        self.lock = threading.RLock()

        self.defaults = {}
        for group in self.GROUPS:
            self.defaults[group] = {}
            for short, gain_name in self.GAIN_NAMES.items():
                param = "~{}_reset_{}".format(group, short)
                default = self.YAML_DEFAULTS[group][gain_name]
                self.defaults[group][gain_name] = float(
                    rospy.get_param(param, default)
                )
                self.validate_gain(gain_name, self.defaults[group][gain_name])

        targets = {
            "xy": rospy.get_param("~xy_target", "/dragon/controller/xy"),
            "z": rospy.get_param("~z_target", "/dragon/controller/z"),
        }
        self.clients = {}
        for group in self.GROUPS:
            rospy.loginfo("Connecting to %s PID server: %s", group.upper(), targets[group])
            self.clients[group] = Client(targets[group], timeout=5)

        self.gains_pub = rospy.Publisher(
            "~current_gains", String, queue_size=1, latch=True
        )

        # The dynamic_reconfigure defaults may differ from the gains originally
        # loaded by the controller from YAML. Synchronize the values at startup.
        # The controller callback processes *one gain field per update*.
        with self.lock:
            for group in self.GROUPS:
                self.apply_gains(group, self.defaults[group])
            self.report_gains("INITIALIZE")

        # Full ROS topic interface, e.g. /dragon_xyz_pid_tuner/z/p_gain.
        for group in self.GROUPS:
            for gain_name in self.GAIN_NAMES.values():
                rospy.Subscriber(
                    "~{}/{}".format(group, gain_name),
                    Float64, self.topic_callback,
                    callback_args=(group, gain_name), queue_size=1
                )

        # Compatibility aliases: the original unprefixed topics adjust XY.
        for gain_name in self.GAIN_NAMES.values():
            rospy.Subscriber(
                "~" + gain_name, Float64, self.topic_callback,
                callback_args=("xy", gain_name), queue_size=1
            )

        self.reset_srv = rospy.Service("~reset", Trigger, self.reset_callback)
        self.get_srv = rospy.Service("~get_gains", Trigger, self.get_callback)
        rospy.loginfo("DRAGON XYZ PID gain tuner is ready.")

    @staticmethod
    def validate_gain(name, value):
        # No tool-specific P/I/D upper/lower bounds. Original PID.cfg governs.
        # Still reject NaN and infinity as invalid numeric inputs.
        if not math.isfinite(float(value)):
            raise ValueError("{} must be a finite number".format(name))

    @staticmethod
    def report_server_adjustment(group, requested, applied):
        for name, value in requested.items():
            actual = applied[name]
            if not math.isclose(float(value), actual, rel_tol=1e-9, abs_tol=1e-9):
                print("  NOTE: {} {} requested {}, server applied {} "
                      "(see original PID.cfg limits).".format(
                          group.upper(), name, value, actual))

    def apply_gains(self, group, gains):
        """Update P, I, and D separately: the C++ callback uses switch(level)."""
        for name in self.GAIN_NAMES.values():
            self.validate_gain(name, float(gains[name]))

        client = self.clients[group]
        client.update_configuration({"pid_control_flag": True})
        for name in ("p_gain", "i_gain", "d_gain"):
            client.update_configuration({name: float(gains[name])})
        self.report_server_adjustment(group, gains, self.read_gains(group))

    def read_gains(self, group):
        config = self.clients[group].get_configuration(timeout=3)
        if config is None:
            raise RuntimeError("Cannot read {} PID configuration".format(group))
        return {name: float(config[name]) for name in self.GAIN_NAMES.values()}

    def report_gains(self, operation):
        lines = []
        for group in self.GROUPS:
            values = self.read_gains(group)
            lines.append(
                "{}: P={:.3f}, I={:.3f}, D={:.3f}".format(
                    group.upper(), values["p_gain"],
                    values["i_gain"], values["d_gain"]
                )
            )
        message = "\n".join(lines)
        print("\n[{}]\n{}\n".format(operation, message), flush=True)
        self.gains_pub.publish(String(data=message))
        return message

    def set_gain(self, group, name, value):
        self.validate_gain(name, value)
        with self.lock:
            client = self.clients[group]
            client.update_configuration({"pid_control_flag": True})
            client.update_configuration({name: float(value)})
            self.report_server_adjustment(
                group, {name: value}, self.read_gains(group)
            )
            return self.report_gains("SET {} {}".format(group.upper(), name))

    def set_all_gains(self, group, p, i, d):
        gains = {"p_gain": p, "i_gain": i, "d_gain": d}
        with self.lock:
            self.apply_gains(group, gains)
            return self.report_gains("SET {} ALL".format(group.upper()))

    def reset_gains(self, group=None):
        groups = self.GROUPS if group is None else (group,)
        with self.lock:
            for g in groups:
                self.apply_gains(g, self.defaults[g])
            return self.report_gains(
                "RESET ALL" if group is None else "RESET {}".format(group.upper())
            )

    def show_gains(self):
        with self.lock:
            return self.report_gains("SHOW")

    def topic_callback(self, msg, args):
        group, gain_name = args
        try:
            self.set_gain(group, gain_name, float(msg.data))
        except Exception as exc:
            rospy.logerr("Could not set %s %s: %s", group, gain_name, exc)

    def reset_callback(self, _request):
        try:
            return TriggerResponse(success=True, message=self.reset_gains())
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def get_callback(self, _request):
        try:
            return TriggerResponse(success=True, message=self.show_gains())
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    @staticmethod
    def print_help():
        print("""
========== DRAGON XYZ PID Gain Tuner ==========

  xy p 3.0          Change XY P gain (X and Y share gains)
  xy i 0.03         Change XY I gain
  xy d 4.0          Change XY D gain
  xy set 3.6 0.03 4 Set XY P, I, D

  z p 2.8           Change Z P gain
  z i 1.2           Change Z I gain
  z d 2.8           Change Z D gain
  z set 2.8 1.2 2.8 Set Z P, I, D

  p 3.0             Shortcut for 'xy p 3.0' (old command)
  i 0.03            Shortcut for XY I
  d 4.0             Shortcut for XY D
  set 3.6 0.03 4    Shortcut for 'xy set ...'

  show              Show both groups
  reset xy          Reset only XY gains
  reset z           Reset only Z gains
  reset             Reset both groups
  help              Show commands
  quit              Exit tuner (gains remain active)

================================================
""")

    def run_console(self):
        self.print_help()
        while not rospy.is_shutdown():
            try:
                tokens = input("xyz-pid> ").strip().lower().split()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not tokens:
                continue

            try:
                command = tokens[0]
                if command in ("quit", "exit", "q"):
                    print("XYZ PID gain tuner stopped.")
                    break
                elif command == "help" and len(tokens) == 1:
                    self.print_help()
                elif command == "show" and len(tokens) == 1:
                    self.show_gains()
                elif command == "reset" and len(tokens) in (1, 2):
                    group = tokens[1] if len(tokens) == 2 else None
                    if group is not None and group not in self.GROUPS:
                        raise ValueError("reset target must be xy or z")
                    self.reset_gains(group)
                else:
                    if command in self.GROUPS:
                        group, rest = command, tokens[1:]
                    else:
                        # Preserve the XY commands from the original tuner.
                        group, rest = "xy", tokens

                    if len(rest) == 2 and rest[0] in self.GAIN_NAMES:
                        self.set_gain(
                            group, self.GAIN_NAMES[rest[0]], float(rest[1])
                        )
                    elif len(rest) == 4 and rest[0] == "set":
                        p, i, d = map(float, rest[1:])
                        self.set_all_gains(group, p, i, d)
                    else:
                        print("Unknown command. Type 'help' for usage.")
            except ValueError as exc:
                print("Invalid input: {}".format(exc))
            except Exception as exc:
                rospy.logerr("Command failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("dragon_xyz_pid_tuner")
    try:
        tuner = XYZPIDGainTuner()
        tuner.run_console()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("XYZ PID tuner failed: %s", exc)
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("XYZ PID tuner stopped")
