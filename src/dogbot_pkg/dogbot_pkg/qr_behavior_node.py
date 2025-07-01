import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import os

class QRBehaviorNode(Node):
    def __init__(self):
        super().__init__('qr_behavior_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_0 = self.create_subscription(PoseStamped, '/marker_0/aruco_marker_0/pose', self.marker_0_callback, 10)
        self.sub_1 = self.create_subscription(PoseStamped, '/marker_1/aruco_marker_1/pose', self.marker_1_callback, 10)
        self.sub_2 = self.create_subscription(PoseStamped, '/marker_2/aruco_marker_2/pose', self.marker_2_callback, 10)
        self.sub_3 = self.create_subscription(PoseStamped, '/marker_3/aruco_marker_3/pose', self.marker_3_callback, 10)
        self.sub_4 = self.create_subscription(PoseStamped, '/marker_4/aruco_marker_4/pose', self.marker_4_callback, 10)

        self.timer = self.create_timer(1.0, self.clear_cmd)

        # Zig-zag control
        self.zigzag_active = False
        self.zigzag_steps = []
        self.zigzag_timer = None

    def marker_0_callback(self, msg: PoseStamped):
        if self.zigzag_active:
            return
        twist = Twist()
        twist.linear.x = 0.3
        self.get_logger().info("📦 Marker 0 detected: Move forward")
        self.cmd_pub.publish(twist)

    def marker_1_callback(self, msg: PoseStamped):
        if self.zigzag_active:
            return
        twist = Twist()
        twist.linear.x = -0.3
        self.get_logger().info("↩️ Marker 1 detected: Move backward")
        self.cmd_pub.publish(twist)

    def marker_2_callback(self, msg: PoseStamped):
        if self.zigzag_active:
            return
        twist = Twist()
        twist.angular.z = 0.5
        self.get_logger().info("🔄 Marker 2 detected: Rotate in place")
        self.cmd_pub.publish(twist)

    def marker_3_callback(self, msg: PoseStamped):
        if self.zigzag_active:
            return
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("🔊 Marker 3 detected: Beep sound!")
        os.system('aplay /usr/share/sounds/alsa/Front_Center.wav &')
        self.cmd_pub.publish(twist)

    def marker_4_callback(self, msg: PoseStamped):
        if self.zigzag_active:
            return

        self.get_logger().info("⚙️ Marker 4 detected: Starting Zig-Zag Mode")

        self.zigzag_active = True
        self.zigzag_steps = [
            {'linear': 0.3, 'angular': 0.0, 'duration': 1.0},
            {'linear': 0.0, 'angular': 0.5, 'duration': 0.5},
            {'linear': 0.3, 'angular': 0.0, 'duration': 1.0},
            {'linear': 0.0, 'angular': -0.5, 'duration': 0.5},
            {'linear': 0.0, 'angular': 0.0, 'duration': 0.0}  # Final stop
        ]
        self.execute_zigzag_step()

    def execute_zigzag_step(self):
        if not self.zigzag_steps:
            self.zigzag_active = False
            self.get_logger().info("✅ Zig-Zag Mode Finished!")
            return

        step = self.zigzag_steps.pop(0)
        twist = Twist()
        twist.linear.x = step['linear']
        twist.angular.z = step['angular']
        self.cmd_pub.publish(twist)

        if step['duration'] > 0:
            self.zigzag_timer = self.create_timer(step['duration'], self.execute_zigzag_step)
        else:
            self.zigzag_active = False
            self.get_logger().info("✅ Zig-Zag Mode Finished!")

    def clear_cmd(self):
        if not self.zigzag_active:
            twist = Twist()
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = QRBehaviorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
