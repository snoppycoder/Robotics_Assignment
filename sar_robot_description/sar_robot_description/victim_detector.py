#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')
        # Define victim positions (must match positions in the SDF world)
        self.victims = {
            'victim_1': (-1.0, -2.0),
            'victim_2': (2.0, 1.0),
            'victim_3': (4.0, -1.0),
        }
        self.detected = {k: False for k in self.victims.keys()}
        self.threshold = 0.8  # meters to consider 'found'

        self.pub = self.create_publisher(String, 'victim_detected', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.get_logger().info('VictimDetector started')

    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        for vid, (vx, vy) in self.victims.items():
            if not self.detected[vid]:
                d = math.hypot(x - vx, y - vy)
                if d <= self.threshold:
                    self.detected[vid] = True
                    out = String()
                    out.data = f'{vid} detected at distance {d:.2f}'
                    self.pub.publish(out)
                    self.get_logger().info(out.data)


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
