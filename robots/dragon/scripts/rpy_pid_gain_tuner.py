#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interactive Roll/Pitch and Yaw PID gain tuner for DRAGON (ROS 1).

This program changes gains through existing dynamic_reconfigure servers.
It does not modify DRAGON's C++ controller or the YAML configuration.

The DRAGON v1_5_202601 configuration shares gains between Roll and Pitch.
Start this node in a terminal (rosrun), not as a background roslaunch node.
"""

import json
import math
import threading

import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


AXES = ("roll_pitch", "yaw")
GAINS = ("p_gain", "i_gain", "d_gain")
GAIN_ALIASES = {"p": "p_gain", "i": "i_gain", "d": "d_gain"}

# Bounds from aerial_robot_control/cfg/PID.cfg, not safe flight ranges.
BOUNDS = {
    "p_gain": (0.0, 50.0),
    "i_gain": (0.0, 20.0),
    "d_gain": (0.0, 50.0),
}

# Defaults from dragon/config/quad/v1_5_202601/control/FullVectoringControlConfig.yaml.
BASELINE = {
    "roll_pitch": {"p_gain": 15.0, "i_gain": 0.3, "d_gain": 5.0},
    "yaw": {"p_gain": 10.0, "i_gain": 0.3, "d_gain": 5.0},
}


class RpyPIDGainTuner:
    def __init__(self):
        self.lock = threading.RLock()
        self.defaults = {
            axis: {
                key: float(rospy.get_param("~reset_{}_{}".format(axis, key), value))
                for key, value in BASELINE[axis].items()
            }
            for axis in AXES
        }
        for axis in AXES:
            self._validate_set(self.defaults[axis])

        self.clients = {}
        for axis in AXES:
            namespace = rospy.get_param(
                "~{}_namespace".format(axis), "/dragon/controller/" + axis
            )
            rospy.loginfo("Connecting to %s ...", namespace)
            self.clients[axis] = Client(namespace, timeout=8)

        # Latched, machine-readable snapshot; also useful for rosbag.
        self.gains_pub = rospy.Publisher(
            "~current_gains", String, queue_size=1, latch=True
        )
        self.get_service = rospy.Service("~get_gains", Trigger, self._get_callback)
        self.reset_service = rospy.Service("~reset", Trigger, self._reset_callback)

        # Read-only startup. Do not change an airborne controller just by
        # launching this tool. Use 'reset' explicitly to apply baselines.
        self.show("STARTUP")
        print("NOTE: Roll and Pitch share one gain set in this DRAGON config.")
        print("      Startup does NOT reset gains. Type 'reset' to apply baselines.")

    @staticmethod
    def _validate_one(key, value):
        low, high = BOUNDS[key]
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(
                "{} must be finite and between {} and {}".format(key, low, high)
            )

    @classmethod
    def _validate_set(cls, values):
        for key in GAINS:
            cls._validate_one(key, float(values[key]))

    def _read_axis(self, axis):
        config = self.clients[axis].get_configuration(timeout=3)
        if config is None:
            raise RuntimeError("Cannot read {} PID configuration".format(axis))
        return {key: float(config[key]) for key in GAINS}

    def _read_all(self):
        return {axis: self._read_axis(axis) for axis in AXES}

    @staticmethod
    def _formatted(axis, values):
        return "{}: P={:.3f}, I={:.3f}, D={:.3f}".format(
            "Roll/Pitch" if axis == "roll_pitch" else "Yaw",
            values["p_gain"], values["i_gain"], values["d_gain"]
        )

    def show(self, label="SHOW", selected=None):
        with self.lock:
            snapshot = self._read_all()
            axes = AXES if selected is None else (selected,)
            print("\n[{}]".format(label))
            for axis in axes:
                print("  " + self._formatted(axis, snapshot[axis]))
            self.gains_pub.publish(String(data=json.dumps(snapshot, sort_keys=True)))
            return snapshot

    def _update_axis(self, axis, values):
        """Validate first; send each gain separately for cfgPidCallback's levels."""
        self._validate_set(values)
        client = self.clients[axis]
        client.update_configuration({"pid_control_flag": True})
        for key in GAINS:
            client.update_configuration({key: float(values[key])})

    def set_one(self, axis, key, value):
        self._validate_one(key, value)
        with self.lock:
            client = self.clients[axis]
            client.update_configuration({"pid_control_flag": True})
            client.update_configuration({key: value})
            return self.show("SET {} {}".format(axis, key), selected=axis)

    def set_group(self, axis, p, i, d):
        values = {"p_gain": p, "i_gain": i, "d_gain": d}
        with self.lock:
            self._update_axis(axis, values)
            return self.show("SET {}".format(axis), selected=axis)

    def reset(self, selected=None):
        axes = AXES if selected is None else (selected,)
        with self.lock:
            for axis in axes:
                self._update_axis(axis, self.defaults[axis])
            return self.show("RESET", selected=selected)

    def _get_callback(self, _request):
        try:
            snapshot = self.show("SERVICE GET")
            return TriggerResponse(True, json.dumps(snapshot, sort_keys=True))
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    def _reset_callback(self, _request):
        try:
            snapshot = self.reset()
            return TriggerResponse(True, json.dumps(snapshot, sort_keys=True))
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    @staticmethod
    def help():
        print("""
========== DRAGON RPY PID Gain Tuner ==========
  rp p 14.0          Set Roll/Pitch P gain (shared)
  rp i 0.3           Set Roll/Pitch I gain (shared)
  rp d 5.0           Set Roll/Pitch D gain (shared)
  rp set 15 0.3 5    Set all Roll/Pitch gains

  yaw p 10.0         Set Yaw P gain
  yaw i 0.3          Set Yaw I gain
  yaw d 5.0          Set Yaw D gain
  yaw set 10 0.3 5   Set all Yaw gains

  show               Show both groups
  show rp            Show Roll/Pitch gains
  show yaw           Show Yaw gains
  reset              Reset both groups to YAML baselines
  reset rp           Reset Roll/Pitch gains only
  reset yaw          Reset Yaw gains only
  help               Show command list
  quit               Exit (last gains remain active)
================================================
""")

    @staticmethod
    def _axis(alias):
        if alias in ("rp", "roll_pitch", "roll-pitch"):
            return "roll_pitch"
        if alias == "yaw":
            return "yaw"
        raise ValueError("Use 'rp' or 'yaw' for the PID group")

    def execute(self, line):
        """Execute one command. Return False only when the user quits."""
        args = line.strip().lower().split()
        if not args:
            return True
        command = args[0]
        if command in ("quit", "exit", "q"):
            if len(args) != 1:
                raise ValueError("Usage: quit")
            return False
        if command == "help":
            self.help()
            return True
        if command == "show":
            if len(args) > 2:
                raise ValueError("Usage: show [rp|yaw]")
            selected = self._axis(args[1]) if len(args) == 2 else None
            self.show(selected=selected)
            return True
        if command == "reset":
            if len(args) > 2:
                raise ValueError("Usage: reset [rp|yaw]")
            selected = self._axis(args[1]) if len(args) == 2 else None
            self.reset(selected=selected)
            return True
        if command in ("rp", "roll_pitch", "roll-pitch", "yaw"):
            axis = self._axis(command)
            if len(args) == 3 and args[1] in GAIN_ALIASES:
                self.set_one(axis, GAIN_ALIASES[args[1]], float(args[2]))
                return True
            if len(args) == 5 and args[1] == "set":
                p, i, d = (float(x) for x in args[2:])
                self.set_group(axis, p, i, d)
                return True
            raise ValueError("Usage: {} <p|i|d> <value>  OR  {} set <p> <i> <d>".format(command, command))
        raise ValueError("Unknown command '{}'. Type 'help'".format(command))

    def run(self):
        self.help()
        while not rospy.is_shutdown():
            try:
                line = input("rpy-pid> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break
            try:
                if not self.execute(line):
                    break
            except ValueError as exc:
                print("Invalid input: {}".format(exc))
            except Exception as exc:
                rospy.logerr("Command failed: %s", exc)
                print("WARNING: A failed multi-gain update may be partially applied.")
        print("RPY PID tuner stopped. DRAGON controller gains are NOT reset.")


def main():
    rospy.init_node("rpy_pid_gain_tuner")
    try:
        tuner = RpyPIDGainTuner()
        tuner.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Cannot start RPY PID tuner: %s", exc)
    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("RPY PID tuner stopped")


if __name__ == "__main__":
    main()
