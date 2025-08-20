#!/usr/bin/env python3
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped

class QRBehaviorNode(Node):
    def __init__(self):
        super().__init__('qr_behavior_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_0 = self.create_subscription(PoseStamped, '/marker_0/aruco_marker_0/pose', self.marker_0_callback, 10)
        self.sub_1 = self.create_subscription(PoseStamped, '/marker_1/aruco_marker_1/pose', self.marker_1_callback, 10)
        self.sub_2 = self.create_subscription(PoseStamped, '/marker_2/aruco_marker_2/pose', self.marker_2_callback, 10)
        self.sub_3 = self.create_subscription(PoseStamped, '/marker_3/aruco_marker_3/pose', self.marker_3_callback, 10)
        self.sub_4 = self.create_subscription(PoseStamped, '/marker_4/aruco_marker_4/pose', self.marker_4_callback, 10)

        # Idle clear
        self.timer = self.create_timer(1.0, self.clear_cmd)

        # Macros
        self.zigzag_active = False
        self.zigzag_steps = []
        self.zigzag_timer = None

        self.spin_active = False
        self.spin_timer = None
        self.spin_end_time = None
        self.spin_speed = 0.6  # rad/s

        self.party_active = False
        self.party_steps = []
        self.party_until = None
        self.party_timer = self.create_timer(0.05, self._party_tick)  # 20 Hz driver

        self.speak("Online. Ready for QR actions.")

    # ---------- voice ----------
    def speak(self, text: str):
        self.get_logger().info(f"🔊 {text}")
        os.system(f'spd-say "{text}" &')

    # ---------- marker callbacks ----------
    def marker_0_callback(self, msg: PoseStamped):
        if self._busy(): return
        tw = Twist(); tw.linear.x = 0.3
        self.speak("Marker zero. Advancing.")
        self.cmd_pub.publish(tw)

    def marker_1_callback(self, msg: PoseStamped):
        if self._busy(): return
        tw = Twist(); tw.linear.x = -0.3
        self.speak("Marker one. Reversing.")
        self.cmd_pub.publish(tw)

    def marker_2_callback(self, msg: PoseStamped):
        if self._busy(): return
        self._start_full_spin()

    def marker_3_callback(self, msg: PoseStamped):
        # Party mode: cancel other macros then run interactive routine
        self._abort_all()
        self._start_party_mode()

    def marker_4_callback(self, msg: PoseStamped):
        if self._busy(): return
        self.speak("Marker four. Zigzag mode.")
        self.zigzag_active = True
        self.zigzag_steps = [
            {'linear': 0.30, 'angular': 0.00, 'duration': 1.0},
            {'linear': 0.00, 'angular': 0.60, 'duration': 0.6},
            {'linear': 0.30, 'angular': 0.00, 'duration': 1.0},
            {'linear': 0.00, 'angular': -0.60, 'duration': 0.6},
            {'linear': 0.00, 'angular': 0.00, 'duration': 0.0}
        ]
        self._exec_zigzag_step()

    # ---------- full 360 spin (Marker 2) ----------
    def _start_full_spin(self):
        self.spin_active = True
        omega = float(self.spin_speed) or 0.6
        dur_sec = abs((2.0 * math.pi) / omega)
        self.spin_end_time = self.get_clock().now() + Duration(seconds=dur_sec)
        self.speak(f"Marker two. Full spin in {dur_sec:.1f} seconds. Do not blink.")
        # 20 Hz spinner
        self.spin_timer = self.create_timer(0.05, self._spin_tick)

    def _spin_tick(self):
        if not self.spin_active: return
        if self.get_clock().now() >= self.spin_end_time:
            self._spin_stop(); return
        tw = Twist(); tw.angular.z = self.spin_speed
        self.cmd_pub.publish(tw)

    def _spin_stop(self):
        if self.spin_timer:
            self.spin_timer.cancel(); self.spin_timer = None
        self.spin_active = False
        self.spin_end_time = None
        self.cmd_pub.publish(Twist())
        self.speak("Spin complete.")

    # ---------- party mode (Marker 3) ----------
    def _start_party_mode(self):
        self.party_active = True
        self.party_steps = []

        # Voice pre-roll
        self.speak("Marker three. Party mode armed.")
        self.speak("Countdown. Three. Two. One. Let's vibe.")

        # Routine:
        # 1) Wiggle spin with tiny hops (2s)
        for _ in range(6):
            self.party_steps.append({'vx': 0.08, 'wz': +1.8, 'dur': 0.15})
            self.party_steps.append({'vx': 0.08, 'wz': -1.8, 'dur': 0.15})
        # 2) Quick 360 flourish (approx 1.2s)
        self.party_steps.append({'vx': 0.00, 'wz': +2.6, 'dur': 1.2})
        # 3) Moonwalk back with micro-oscillation (1.5s)
        for _ in range(5):
            self.party_steps.append({'vx': -0.12, 'wz': +0.6, 'dur': 0.15})
            self.party_steps.append({'vx': -0.12, 'wz': -0.6, 'dur': 0.15})
        # 4) Pose + quip
        self.party_steps.append({'vx': 0.00, 'wz': 0.00, 'dur': 0.0})

        # Prime first step window
        self.party_until = None
        self.speak("Disclaimer. If you laugh, you owe me a firmware upgrade.")

    def _party_tick(self):
        if not self.party_active:
            return

        now = self.get_clock().now()
        # Start or advance step window
        if self.party_until is None:
            self._party_next_step()
            return

        # If step window elapsed, advance
        if now >= self.party_until:
            self._party_next_step()
            return

        # Keep publishing current command at 20 Hz (timer period)
        if hasattr(self, "_party_current"):
            tw = Twist()
            tw.linear.x = self._party_current['vx']
            tw.angular.z = self._party_current['wz']
            self.cmd_pub.publish(tw)

    def _party_next_step(self):
        if not self.party_steps:
            self.party_active = False
            self.cmd_pub.publish(Twist())
            self.speak("Party mode complete. Thank you, I am here all week.")
            return

        step = self.party_steps.pop(0)
        self._party_current = step
        dur = float(step['dur'])
        if dur <= 0.0:
            # Final pose
            self.party_until = None
            self.party_active = False
            self.cmd_pub.publish(Twist())
            self.speak("Freeze frame. Screenshot this swagger.")
            return

        # Set new window
        self.party_until = self.get_clock().now() + Duration(seconds=dur)

    # ---------- zigzag ----------
    def _exec_zigzag_step(self):
        if not self.zigzag_steps:
            self.zigzag_active = False
            self.speak("Zigzag complete.")
            return
        s = self.zigzag_steps.pop(0)
        tw = Twist(); tw.linear.x = s['linear']; tw.angular.z = s['angular']
        self.cmd_pub.publish(tw)
        if s['duration'] > 0:
            self.zigzag_timer = self.create_timer(s['duration'], self._exec_zigzag_step)
        else:
            self.zigzag_active = False
            self.speak("Zigzag complete.")

    # ---------- helpers ----------
    def _busy(self) -> bool:
        return self.zigzag_active or self.spin_active or self.party_active

    def _abort_all(self):
        # Cancel zigzag
        if self.zigzag_timer: self.zigzag_timer.cancel(); self.zigzag_timer = None
        self.zigzag_active = False
        self.zigzag_steps = []
        # Cancel spin
        if self.spin_timer: self.spin_timer.cancel(); self.spin_timer = None
        self.spin_active = False; self.spin_end_time = None
        # Stop motion
        self.cmd_pub.publish(Twist())

    # ---------- idle clear ----------
    def clear_cmd(self):
        if not self._busy():
            self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = QRBehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
