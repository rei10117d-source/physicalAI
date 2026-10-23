#!/usr/bin/env python3
"""ROS 2 Humble odometry recorder. Independent of pai_nav."""
import argparse
import csv
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

COLUMNS = ['sim_time_s','elapsed_s','x_m','y_m','yaw_deg','step_m','cumulative_distance_m','distance_to_goal_m']

def yaw_degrees(q):
    return math.degrees(math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

class Recorder(Node):
    def __init__(self, args):
        super().__init__('pai_trajectory_recorder')
        self.args = args
        self.path = Path(args.output).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.file.flush()
        self.initial_stamp = None
        self.last_stamp = None
        self.last_xy = None
        self.total = 0.0
        self.samples = 0
        self.finished = False
        self.goal_reached = False
        self.last_elapsed = 0.0
        self.create_subscription(Odometry, args.topic, self.on_odom, 10)
        self.get_logger().info(f'Recording {args.topic} to {self.path}')
        self.get_logger().info('Start before launching pai_nav. Ctrl+C to finish; goal arrival stops recording automatically.')

    def on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # Skip duplicate or out-of-order simulator stamps.
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        if self.initial_stamp is None:
            self.initial_stamp = stamp
        step = math.hypot(x-self.last_xy[0],y-self.last_xy[1]) if self.last_xy is not None else 0.0
        # Reject implausible jumps from resets / transport glitches.
        if step > self.args.max_step:
            self.get_logger().warning(f'Odom jump {step:.2f}m ignored; new segment begins.')
            step = 0.0
        self.total += step
        elapsed = stamp - self.initial_stamp
        remaining = math.hypot(self.args.goal_x-x, self.args.goal_y-y)
        self.writer.writerow([f'{stamp:.6f}', f'{elapsed:.6f}',f'{x:.6f}',f'{y:.6f}',
                              f'{yaw_degrees(msg.pose.pose.orientation):.3f}',f'{step:.6f}',
                              f'{self.total:.6f}',f'{remaining:.6f}'])
        self.samples += 1
        self.last_xy = (x,y)
        self.last_stamp = stamp
        self.last_elapsed = elapsed
        if self.samples % 10 == 0:
            self.file.flush()
        if self.samples % 100 == 0:
            self.get_logger().info(f'{elapsed:.1f}s | path={self.total:.2f}m | remaining={remaining:.2f}m')
        if remaining < self.args.goal_radius:
            self.goal_reached = True
            self.finished = True
            self.get_logger().info(f'GOAL DETECTED: elapsed={elapsed:.2f}s, path={self.total:.2f}m')

    def close(self):
        if not self.file.closed:
            self.file.flush()
            self.file.close()
        self.get_logger().info(f'CSV saved: {self.path} | samples={self.samples}, sim_elapsed={self.last_elapsed:.2f}s, path={self.total:.2f}m, goal={self.goal_reached}')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='~/pai_ws/results/trajectory.csv')
    parser.add_argument('--topic', default='/odom')
    parser.add_argument('--goal-x', type=float, default=4.0)
    parser.add_argument('--goal-y', type=float, default=0.0)
    parser.add_argument('--goal-radius', type=float, default=0.35)
    parser.add_argument('--max-step', type=float, default=1.0, help='Ignore odom position jumps larger than this many meters')
    args = parser.parse_args()
    rclpy.init()
    node = Recorder(args)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
