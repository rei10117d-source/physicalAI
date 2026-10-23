"""ROS 2 differential-drive navigation: learned decisions + stable avoidance phases."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

from .policy import KNNPolicy, teacher


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AutonomousNav(Node):
    def __init__(self):
        super().__init__('pai_autonomous_nav')
        self.declare_parameter('mode', 'ai')
        self.declare_parameter('goal_x', 4.0)
        self.declare_parameter('goal_y', 0.0)
        self.mode = self.get_parameter('mode').value
        self.goal = (float(self.get_parameter('goal_x').value),
                     float(self.get_parameter('goal_y').value))
        model_path = os.path.join(get_package_share_directory('pai_nav'),
                                  'models', 'knn_model.json')
        self.policy = KNNPolicy.from_file(model_path)
        self.scan = None
        self.pose = None
        self.last_odom = self.get_clock().now()
        self.last_scan = self.get_clock().now()
        self.finished = False

        # GOAL -> TURN -> PASS -> GOAL.  Emergency STOP overrides each phase.
        self.phase = 'GOAL'
        self.turn_sign = 1.0
        self.phase_started = 0.0
        self.turn_start_yaw = 0.0
        self.last_turn_yaw = 0.0
        self.turn_progress = 0.0
        self.pass_start_xy = None
        self.clear_count = 0
        self.avoid_cooldown_until = 0.0
        self.last_log = -100.0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_timer(0.1, self.step)
        self.get_logger().info(f'PAI navigation mode={self.mode}, goal={self.goal}')

    def on_scan(self, msg):
        self.scan = msg
        self.last_scan = self.get_clock().now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                         1 - 2 * (q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)
        self.last_odom = self.get_clock().now()

    def sector(self, angle, width=math.radians(45)):
        scan = self.scan
        readings = []
        for i, value in enumerate(scan.ranges):
            ray = scan.angle_min + i * scan.angle_increment
            if abs(wrap(ray - angle)) < width / 2:
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                    readings.append(value)
        return min(readings) if readings else scan.range_max

    def stop(self):
        self.pub.publish(Twist())

    def set_phase(self, name, t):
        if name != self.phase:
            self.phase = name
            self.phase_started = t
            self.get_logger().info(f'Navigation phase: {name}')

    def step(self):
        if self.finished:
            self.stop()
            return
        now = self.get_clock().now()
        if (self.scan is None or self.pose is None or
                (now - self.last_scan).nanoseconds > 1_000_000_000 or
                (now - self.last_odom).nanoseconds > 1_000_000_000):
            self.stop()
            return

        # The controller uses simulation time, not wall time (WSL can run slowly).
        t = now.nanoseconds / 1e9
        x, y, yaw = self.pose
        distance = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if distance < 0.35:
            self.finished = True
            self.stop()
            self.get_logger().info('GOAL REACHED: navigation stopped safely')
            return

        desired = math.atan2(self.goal[1] - y, self.goal[0] - x)
        err = wrap(desired - yaw)
        front = self.sector(0.0, math.radians(40))
        front_left = self.sector(math.pi / 4, math.radians(45))
        front_right = self.sector(-math.pi / 4, math.radians(45))
        left = self.sector(math.pi / 2, math.radians(50))
        right = self.sector(-math.pi / 2, math.radians(50))
        cmd = Twist()

        # Physical safety guard: hard stop before a very close obstacle.
        # Rotating in place is allowed; straight motion is not.
        if self.phase == 'GOAL':
            features = [min(front / 2.0, 1.0),
                        min(self.sector(math.pi/3) / 2.0, 1.0),
                        min(self.sector(-math.pi/3) / 2.0, 1.0),
                        err / math.pi]
            action = self.policy.predict(features) if self.mode == 'ai' else teacher(*features)
            # Safety threshold is applied to either policy mode.
            if front < 0.75 or (action in (1, 2) and front < 1.1 and t >= self.avoid_cooldown_until):
                if front_left < 0.35 and front_right < 0.35:
                    self.stop()
                    return
                if action == 1:
                    sign = 1.0
                elif action == 2:
                    sign = -1.0
                else:
                    sign = 1.0 if front_left >= front_right else -1.0
                # Do not turn toward a substantially more obstructed side.
                if sign > 0 and front_left + 0.3 < front_right:
                    sign = -1.0
                if sign < 0 and front_right + 0.3 < front_left:
                    sign = 1.0
                self.turn_sign = sign
                self.turn_start_yaw = yaw
                self.last_turn_yaw = yaw
                self.turn_progress = 0.0
                self.set_phase('TURN', t)
            else:
                # A direct route to the goal is preferred when the corridor is clear.
                cmd.linear.x = 0.20 if abs(err) < 0.55 else 0.08
                cmd.angular.z = max(-0.45, min(0.45, 1.2 * err))
                if front < 0.30:
                    cmd.linear.x = 0.0
                self.pub.publish(cmd)
                return

        if self.phase == 'TURN':
            # Rotate by around 55 degrees, keeping the original avoidance sign.
            delta = wrap(yaw - self.last_turn_yaw)
            self.turn_progress += self.turn_sign * delta
            self.last_turn_yaw = yaw
            turned = self.turn_progress
            if turned >= 0.95 and front > 0.45:
                self.pass_start_xy = (x, y)
                self.clear_count = 0
                self.set_phase('PASS', t)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_sign * 0.50
                if t - self.phase_started > 7.0:
                    # Stop rather than endlessly spinning in an unrecoverable case.
                    self.get_logger().warn(
                        f'TURN TIMEOUT: turned={math.degrees(turned):.1f}deg, '
                        f'front={front:.2f}m, '
                        f'yaw={math.degrees(yaw):.1f}deg, '
                        f'start={math.degrees(self.turn_start_yaw):.1f}deg, '
                        f'sign={self.turn_sign}'
                    )
                    self.finished = True
                    self.stop()
                    return
                self.pub.publish(cmd)
                return

        if self.phase == 'PASS':
            # Drive along the obstacle; permit a small steering bias AWAY from it.
            nearest_side = right if self.turn_sign > 0 else left
            far_enough = front > 0.40 and nearest_side > 0.90
            self.clear_count = self.clear_count + 1 if far_enough else 0
            traveled = math.hypot(x - self.pass_start_xy[0],
                                  y - self.pass_start_xy[1]) if self.pass_start_xy else 0.0
            if traveled > 0.90 and front > 0.75 and nearest_side > 0.90:
                self.set_phase('GOAL', t)
                self.avoid_cooldown_until = t + 1.0
            else:
                if front < 0.28:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.turn_sign * 0.4
                else:
                    cmd.linear.x = 0.18
                    cmd.angular.z = 0.0
                if t - self.phase_started > 18.0:
                    self.get_logger().warn(
                        f'PASS TIMEOUT: traveled={traveled:.2f}m, '
                        f'front={front:.2f}m, '
                        f'left={left:.2f}m, right={right:.2f}m, '
                        f'nearest_side={nearest_side:.2f}m, '
                        f'clear_count={self.clear_count}'
                    )
                    self.finished = True
                    self.stop()
                    return
                self.pub.publish(cmd)
                return

        # Phase just returned to GOAL: publish a safe stop for one cycle.
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS shutdown can invalidate a publisher before the final callback.
        if rclpy.ok():
            try:
                node.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
