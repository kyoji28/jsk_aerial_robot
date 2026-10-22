#!/usr/bin/env python3

import math
import threading

import rospy
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, TriggerResponse
from dynamic_reconfigure.client import Client


class PIDGainTuner:

    def __init__(self):
        self.lock = threading.RLock()

        # 変更対象：DRAGONのXY位置制御PID
        target = rospy.get_param(
            "~target", "/dragon/controller/xy"
        )

        # リセット時の初期値
        self.defaults = {
            "p_gain": rospy.get_param("~reset_p", 3.6),
            "i_gain": rospy.get_param("~reset_i", 0.03),
            "d_gain": rospy.get_param("~reset_d", 4.0),
        }

        # PID.cfgで定義された設定範囲
        self.limits = {
            "p_gain": (0.0, 50.0),
            "i_gain": (0.0, 20.0),
            "d_gain": (0.0, 50.0),
        }

        # 現在値の配信
        self.gains_pub = rospy.Publisher(
            "~current_gains",
            String,
            queue_size=1,
            latch=True
        )

        # 既存のPIDコントローラに接続
        self.client = Client(target, timeout=5)

        # 起動時に初期値を適用
        self.apply_gains(self.defaults)
        self.report_gains("INITIALIZE")

        # ROSトピックからの変更も受け付ける
        for name in ("p_gain", "i_gain", "d_gain"):
            rospy.Subscriber(
                "~" + name,
                Float64,
                self.topic_callback,
                callback_args=name,
                queue_size=1
            )

        # ROSサービス
        self.reset_srv = rospy.Service(
            "~reset", Trigger, self.reset_callback
        )

        self.get_srv = rospy.Service(
            "~get_gains", Trigger, self.get_callback
        )

    def validate_gain(self, name, value):
        low, high = self.limits[name]

        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(
                "{} must be between {} and {}".format(
                    name, low, high
                )
            )

    def apply_gains(self, gains):
        # 全ての入力値を先に検証
        for name in ("p_gain", "i_gain", "d_gain"):
            self.validate_gain(name, float(gains[name]))

        # ゲイン変更を有効化
        self.client.update_configuration({
            "pid_control_flag": True
        })

        # 既存コードの仕様に合わせて1項目ずつ送信
        for name in ("p_gain", "i_gain", "d_gain"):
            self.client.update_configuration({
                name: float(gains[name])
            })

    def read_gains(self):
        config = self.client.get_configuration(timeout=3)

        if config is None:
            raise RuntimeError("Cannot read PID gains")

        return {
            "p_gain": config["p_gain"],
            "i_gain": config["i_gain"],
            "d_gain": config["d_gain"],
        }

    def report_gains(self, operation):
        gains = self.read_gains()

        message = "P={:.3f}, I={:.3f}, D={:.3f}".format(
            gains["p_gain"],
            gains["i_gain"],
            gains["d_gain"]
        )

        print("[{}] {}".format(operation, message), flush=True)
        self.gains_pub.publish(String(data=message))

        return message

    def set_gain(self, name, value):
        with self.lock:
            self.validate_gain(name, value)

            self.client.update_configuration({
                "pid_control_flag": True
            })

            self.client.update_configuration({
                name: value
            })

            return self.report_gains(
                "SET " + name[0].upper()
            )

    def set_all_gains(self, p, i, d):
        gains = {
            "p_gain": p,
            "i_gain": i,
            "d_gain": d
        }

        with self.lock:
            self.apply_gains(gains)
            return self.report_gains("SET ALL")

    def reset_gains(self):
        with self.lock:
            self.apply_gains(self.defaults)
            return self.report_gains("RESET")

    def show_gains(self):
        with self.lock:
            return self.report_gains("GET")

    # --- ROSトピック用 ---

    def topic_callback(self, msg, name):
        try:
            self.set_gain(name, msg.data)
        except Exception as exc:
            rospy.logerr("Gain update failed: %s", exc)

    # --- ROSサービス用 ---

    def reset_callback(self, request):
        try:
            message = self.reset_gains()
            return TriggerResponse(True, message)
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    def get_callback(self, request):
        try:
            message = self.show_gains()
            return TriggerResponse(True, message)
        except Exception as exc:
            return TriggerResponse(False, str(exc))

    # --- キーボード操作用 ---

    def print_help(self):
        print("""
========== DRAGON PID Gain Tuner ==========

  p <value>         Change P gain
  i <value>         Change I gain
  d <value>         Change D gain
  set <p> <i> <d>   Set all gains

  show              Show current gains
  reset             Reset to default gains
  help              Show this help
  quit              Exit tuner

===========================================
""")

    def run_console(self):
        self.print_help()

        while not rospy.is_shutdown():
            try:
                line = input("pid> ").strip()

            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            args = line.lower().split()
            command = args[0]

            try:
                gain_names = {
                    "p": "p_gain",
                    "i": "i_gain",
                    "d": "d_gain"
                }

                if command in gain_names:
                    if len(args) != 2:
                        print("Usage: {} <value>".format(command))
                        continue

                    value = float(args[1])
                    self.set_gain(gain_names[command], value)

                elif command == "set":
                    if len(args) != 4:
                        print("Usage: set <p> <i> <d>")
                        continue

                    p, i, d = map(float, args[1:4])
                    self.set_all_gains(p, i, d)

                elif command == "show":
                    self.show_gains()

                elif command == "reset":
                    self.reset_gains()

                elif command == "help":
                    self.print_help()

                elif command in ("quit", "exit", "q"):
                    print("PID gain tuner stopped.")
                    break

                else:
                    print("Unknown command. Type 'help'.")

            except ValueError as exc:
                print("Invalid input:", exc)

            except Exception as exc:
                rospy.logerr("Command failed: %s", exc)


if __name__ == "__main__":
    rospy.init_node("dragon_pid_tuner")

    try:
        tuner = PIDGainTuner()
        tuner.run_console()

    except rospy.ROSInterruptException:
        pass

    except Exception as exc:
        rospy.logerr("PID tuner startup failed: %s", exc)

    finally:
        if not rospy.is_shutdown():
            rospy.signal_shutdown("PID tuner stopped")
