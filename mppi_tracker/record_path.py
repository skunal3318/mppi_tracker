#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

from rclpy.node import Node
from visualization_msgs.msg import Marker


class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')

        self.declare_parameter('output_file', '/tmp/recorded_path.csv')
        self.declare_parameter('min_spacing', 0.1)

        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.min_spacing = self.get_parameter('min_spacing').get_parameter_value().double_value

        self.last_point = None
        self.points = []

        os.makedirs(os.path.dirname(self.output_file) or '.', exist_ok=True)
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.path_pub = self.create_publisher(Path, '/recorded_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/recorded_path_marker', 10)

        self.get_logger().info(
            f'Recording path to {self.output_file}, min spacing {self.min_spacing}m. '
            'Drive the robot to build the path via teleop terminal.'
        )

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_point is None:
            self._record_point(x, y)
            return

        dx = x - self.last_point[0]
        dy = y - self.last_point[1]
        dist = math.hypot(dx, dy)

        if dist >= self.min_spacing:
            self._record_point(x, y)

    def _record_point(self, x, y):
        self.last_point = (x, y)
        self.points.append((x, y))
        self.csv_writer.writerow([f'{x:.4f}', f'{y:.4f}'])
        self.csv_file.flush()
        self._publish_visuals()

        if len(self.points) % 10 == 0:
            self.get_logger().info(f'Recorded {len(self.points)} points ...')

    def _publish_visuals(self):
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in self.points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'recorded_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for (x, y) in self.points:
            p = PoseStamped().pose.position
            p.x, p.y, p.z = x, y, 0.0
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(
            f'Saved {len(self.points)} points to {self.output_file}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
