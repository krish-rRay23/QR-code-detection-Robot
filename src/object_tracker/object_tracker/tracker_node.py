#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from yolov5_msgs.msg import DetectionArray  # Adjust this if your detection message type is different
import tf2_ros
import tf_transformations

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        # Target object class to track
        self.target_class = 'bottle'  # Example: change to the object you want

        # Create tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to YOLOv5 detection topic
        self.create_subscription(
            DetectionArray,
            '/yolov5/detections',
            self.detection_callback,
            10)

        self.saved_pose = None

    def detection_callback(self, msg):
        for detection in msg.detections:
            if detection.class_name == self.target_class:
                self.get_logger().info(f'{self.target_class} detected!')

                try:
                    now = rclpy.time.Time()
                    transform = self.tf_buffer.lookup_transform('map', 'base_link', now)

                    position = transform.transform.translation
                    self.saved_pose = (position.x, position.y)

                    self.get_logger().info(f'Saved location: x={position.x}, y={position.y}')

                except Exception as e:
                    self.get_logger().error(f'TF lookup failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
