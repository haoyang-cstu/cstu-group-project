import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        self.declare_parameter('forward_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.9)
        self.declare_parameter('heading_gain', 1.8)
        self.declare_parameter('waypoint_tolerance', 0.35)
        self.declare_parameter('stop_duration', 2.0)
        self.declare_parameter('stop_cooldown', 4.0)
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.sub = self.create_subscription(Bool, '/stop_detected', self.cb_stop, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.cb_odom, 10)
        self.stopped = False
        self.stop_until = 0.0
        self.ignore_stops_until = 0.0
        self.pose = None
        self.route_done = False
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.heading_gain = self.get_parameter('heading_gain').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.stop_duration = self.get_parameter('stop_duration').get_parameter_value().double_value
        self.stop_cooldown = self.get_parameter('stop_cooldown').get_parameter_value().double_value
        self.waypoints = [
            (5.35, 0.0),
            (5.35, -4.6),
        ]
        self.turn_headings = [
            -math.pi / 2.0,
            None,
        ]
        self.current_waypoint = 0
        self.turn_target_yaw = None
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info(f'Publishing auto-drive commands to {cmd_vel_topic}; following odom from {odom_topic}')

    def cb_stop(self, msg: Bool):
        now = self.now_sec()
        if msg.data and not self.stopped and now >= self.ignore_stops_until:
            self.get_logger().info('Stop sign detected: stopping for 2s')
            self.stopped = True
            self.stop_until = now + self.stop_duration
            self.ignore_stops_until = self.stop_until + self.stop_cooldown

    def cb_odom(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self.yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.pose = (position.x, position.y, yaw)

    def update(self):
        now = self.now_sec()

        if self.stopped:
            if now >= self.stop_until:
                self.stopped = False
                self.get_logger().info('Resuming motion')
            else:
                self.publish_cmd(0.0, 0.0)
                return

        if self.pose is None:
            self.publish_cmd(self.forward_speed, 0.0)
            return

        if self.route_done:
            self.publish_cmd(0.0, 0.0)
            return

        x, y, yaw = self.pose

        if self.turn_target_yaw is not None:
            heading_error = self.wrap_angle(self.turn_target_yaw - yaw)
            if abs(heading_error) < 0.08:
                self.turn_target_yaw = None
                if self.current_waypoint < len(self.waypoints) - 1:
                    self.current_waypoint += 1
                    target_x, target_y = self.waypoints[self.current_waypoint]
                    self.get_logger().info(
                        f'90 degree right turn complete; heading to waypoint {self.current_waypoint + 1}: '
                        f'({target_x:.1f}, {target_y:.1f})'
                    )
                else:
                    self.route_done = True
                    self.get_logger().info('Route complete')
                self.publish_cmd(0.0, 0.0)
                return

            angular_z = self.clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)
            self.publish_cmd(0.0, angular_z)
            return

        target_x, target_y = self.waypoints[self.current_waypoint]
        distance = math.hypot(target_x - x, target_y - y)

        if distance < self.waypoint_tolerance:
            target_heading = self.turn_headings[self.current_waypoint]
            if target_heading is not None:
                self.turn_target_yaw = target_heading
                self.get_logger().info('At corner: making a 90 degree right turn')
                self.publish_cmd(0.0, 0.0)
                return

            if self.current_waypoint < len(self.waypoints) - 1:
                self.current_waypoint += 1
                target_x, target_y = self.waypoints[self.current_waypoint]
                distance = math.hypot(target_x - x, target_y - y)
                self.get_logger().info(f'Heading to waypoint {self.current_waypoint + 1}: ({target_x:.1f}, {target_y:.1f})')
            else:
                self.route_done = True
                self.get_logger().info('Route complete')
                self.publish_cmd(0.0, 0.0)
                return

        desired_heading = math.atan2(target_y - y, target_x - x)
        heading_error = self.wrap_angle(desired_heading - yaw)
        angular_z = self.clamp(self.heading_gain * heading_error, -self.max_angular_speed, self.max_angular_speed)
        turn_scale = max(0.25, 1.0 - min(abs(heading_error), 1.2) / 1.2 * 0.75)
        linear_x = self.forward_speed * turn_scale

        self.publish_cmd(linear_x, angular_z)

    def publish_cmd(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.pub.publish(twist)

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def yaw_from_quaternion(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(value, upper))


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
