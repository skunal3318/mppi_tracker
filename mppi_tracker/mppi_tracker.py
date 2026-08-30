#!/usr/bin/env python3
import csv
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf_transformations import euler_from_quaternion


def load_path_csv(path_file):
    pts = []
    with open(path_file, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 2:
                continue
            pts.append((float(row[0]), float(row[1])))
    return np.array(pts, dtype=float)


class MPPITracker(Node):
    def __init__(self):
        super().__init__('mppi_tracker')

        self.declare_parameter('path_file','/tmp/test_path.csv')
        self.declare_parameter('num_samples',500)
        self.declare_parameter('horizon',30)
        self.declare_parameter('dt',0.1)
        self.declare_parameter('desired_speed',0.2)
        self.declare_parameter('max_linear_vel',0.22)
        self.declare_parameter('max_angular_vel',1.5)
        self.declare_parameter('noise_std_v',0.08)
        self.declare_parameter('noise_std_w',0.6)
        self.declare_parameter('lambda_temp',1.0)
        self.declare_parameter('w_path',15.0)
        self.declare_parameter('w_control',0.05)
        self.declare_parameter('w_obstacle',1200.0)
        self.declare_parameter('safety_radius',0.45)
        self.declare_parameter('scan_max_range',3.5)
        self.declare_parameter('scan_downsample',5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('nearest_search_window',10)
        self.declare_parameter('control_rate', 20.0)

        self.path_file = self.get_parameter('path_file').get_parameter_value().string_value
        self.K = self.get_parameter('num_samples').get_parameter_value().integer_value
        self.H = self.get_parameter('horizon').get_parameter_value().integer_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.desired_speed = self.get_parameter('desired_speed').get_parameter_value().double_value
        self.max_v = self.get_parameter('max_linear_vel').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_vel').get_parameter_value().double_value
        self.noise_std_v = self.get_parameter('noise_std_v').get_parameter_value().double_value
        self.noise_std_w = self.get_parameter('noise_std_w').get_parameter_value().double_value
        self.lambda_temp = self.get_parameter('lambda_temp').get_parameter_value().double_value
        self.w_path = self.get_parameter('w_path').get_parameter_value().double_value
        self.w_control = self.get_parameter('w_control').get_parameter_value().double_value
        self.w_obstacle = self.get_parameter('w_obstacle').get_parameter_value().double_value
        self.safety_radius = self.get_parameter('safety_radius').get_parameter_value().double_value
        self.scan_max_range = self.get_parameter('scan_max_range').get_parameter_value().double_value
        self.scan_downsample = self.get_parameter('scan_downsample').get_parameter_value().integer_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.nearest_search_window = self.get_parameter('nearest_search_window').get_parameter_value().integer_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value

        self.path = load_path_csv(self.path_file)
        if len(self.path) < 2:
            self.get_logger().error(f'Path file {self.path_file} has fewer than 2 points!')
        self.get_logger().info(f'Loaded path with {len(self.path)} points from {self.path_file}')

        self.pose = None
        self.nominal_u = np.zeros((self.H, 2))
        self.goal_reached = False
        self.last_nearest_idx = 0

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.latest_scan = None
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.recorded_path_pub = self.create_publisher(Marker, '/mppi/recorded_path_marker', 10)
        self.traversed_path_pub = self.create_publisher(Marker, '/mppi/traversed_path_marker', 10)
        self.traversed_points = []
        self.last_traversed_point = None

        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info('MPPI tracker started')
        self.publish_recorded_path_marker()
        self.marker_timer = self.create_timer(1.0, self.publish_recorded_path_marker)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])

        x, y = self.pose[0], self.pose[1]
        if self.last_traversed_point is None or \
                np.hypot(x - self.last_traversed_point[0], y - self.last_traversed_point[1]) >= 0.05:
            self.last_traversed_point = (x, y)
            self.traversed_points.append((x, y))
            self.publish_traversed_path_marker()

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_obstacle_points_world(self):
        if self.latest_scan is None or self.pose is None:
            return np.empty((0, 2))

        scan = self.latest_scan
        ranges = np.array(scan.ranges[::self.scan_downsample])
        angles = scan.angle_min + np.arange(len(scan.ranges))[::self.scan_downsample] * scan.angle_increment

        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < min(scan.range_max, self.scan_max_range))
        ranges = ranges[valid]
        angles = angles[valid]
        if len(ranges) == 0:
            return np.empty((0, 2))

        robot_x, robot_y, robot_theta = self.pose
        local_x = ranges * np.cos(angles)
        local_y = ranges * np.sin(angles)
        world_x = robot_x + local_x * np.cos(robot_theta) - local_y * np.sin(robot_theta)
        world_y = robot_y + local_x * np.sin(robot_theta) + local_y * np.cos(robot_theta)
        return np.stack([world_x, world_y], axis=1)

    def build_local_reference(self, nearest_idx):
        step_dist = self.desired_speed * self.dt
        ref = np.zeros((self.H, 2))
        idx = nearest_idx
        pos = self.path[idx].copy()
        remaining = 0.0
        for t in range(self.H):
            dist_to_go = step_dist
            while dist_to_go > 0 and idx < len(self.path) - 1:
                seg = self.path[idx + 1] - self.path[idx]
                seg_len = np.linalg.norm(seg)
                if seg_len < 1e-6:
                    idx += 1
                    continue
                if remaining + dist_to_go < seg_len:
                    remaining += dist_to_go
                    pos = self.path[idx] + seg * (remaining / seg_len)
                    dist_to_go = 0.0
                else:
                    dist_to_go -= (seg_len - remaining)
                    idx += 1
                    remaining = 0.0
                    pos = self.path[idx].copy()
            ref[t] = pos
        return ref

    def rollout(self, x0, controls):
        K = controls.shape[0]
        states = np.zeros((K, self.H, 3))
        cx = np.full(K, x0[0])
        cy = np.full(K, x0[1])
        ctheta = np.full(K, x0[2])
        for t in range(self.H):
            v = controls[:, t, 0]
            w = controls[:, t, 1]
            cx = cx + v * np.cos(ctheta) * self.dt
            cy = cy + v * np.sin(ctheta) * self.dt
            ctheta = ctheta + w * self.dt
            states[:, t, 0] = cx
            states[:, t, 1] = cy
            states[:, t, 2] = ctheta
        return states

    def control_loop(self):
        if self.pose is None or self.goal_reached or len(self.path) < 2:
            return

        search_start = max(0, self.last_nearest_idx - 5)
        search_end = min(len(self.path), self.last_nearest_idx + self.nearest_search_window)
        window = self.path[search_start:search_end]
        dists = np.linalg.norm(window - self.pose[:2], axis=1)
        nearest_idx = search_start + int(np.argmin(dists))
        self.last_nearest_idx = nearest_idx

        dist_to_goal = np.linalg.norm(self.path[-1] - self.pose[:2])
        if nearest_idx >= len(self.path) - 2 and dist_to_goal < self.goal_tolerance:
            self.goal_reached = True
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('Goal reached, stopping the tracker......')
            return

        ref = self.build_local_reference(nearest_idx)

        noise_v = np.random.normal(0.0, self.noise_std_v, size=(self.K, self.H))
        noise_w = np.random.normal(0.0, self.noise_std_w, size=(self.K, self.H))
        controls = np.zeros((self.K, self.H, 2))
        controls[:, :, 0] = np.clip(self.nominal_u[:, 0] + noise_v, 0.0, self.max_v)
        controls[:, :, 1] = np.clip(self.nominal_u[:, 1] + noise_w, -self.max_w, self.max_w)

        states = self.rollout(self.pose, controls)

        diff = states[:, :, 0:2] - ref[np.newaxis, :, :]
        path_cost = np.sum(diff[:, :, 0] ** 2 + diff[:, :, 1] ** 2, axis=1)
        control_cost = np.sum(controls[:, :, 0] ** 2 + controls[:, :, 1] ** 2, axis=1)

        obstacle_points = self.get_obstacle_points_world()
        if len(obstacle_points) > 0:
            rollout_xy = states[:, :, 0:2]
            deltas = rollout_xy[:, :, np.newaxis, :] - obstacle_points[np.newaxis, np.newaxis, :, :]
            dists = np.sqrt(np.sum(deltas ** 2, axis=-1))
            min_dists = np.min(dists, axis=-1)
            violation = np.maximum(0.0, self.safety_radius - min_dists)
            obstacle_cost = np.sum(violation ** 2, axis=1)
        else:
            obstacle_cost = np.zeros(self.K)

        S = self.w_path * path_cost + self.w_control * control_cost + self.w_obstacle * obstacle_cost

        beta = np.min(S)
        weights = np.exp(-(S - beta) / self.lambda_temp)
        weights /= (np.sum(weights) + 1e-10)

        self.nominal_u = np.tensordot(weights, controls, axes=(0, 0))

        v_cmd, w_cmd = self.nominal_u[0]
        self.publish_cmd(float(v_cmd), float(w_cmd))

        self.nominal_u = np.roll(self.nominal_u, -1, axis=0)
        self.nominal_u[-1] = self.nominal_u[-2]

    def publish_recorded_path_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mppi_recorded_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for (x, y) in self.path:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            marker.points.append(p)
        self.recorded_path_pub.publish(marker)

    def publish_traversed_path_marker(self):
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mppi_traversed_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for (x, y) in self.traversed_points:
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            marker.points.append(p)
        self.traversed_path_pub.publish(marker)

    def publish_cmd(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPPITracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
