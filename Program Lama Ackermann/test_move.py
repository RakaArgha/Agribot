# test_move.p
#!/usr/bin/env python3
"""
Publisher TwistStamped dengan header.stamp yang benar (mengikuti /clock simulasi).
Jalankan: python3 test_move.py
Tekan Ctrl+C untuk berhenti.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped


class TestMover(Node):
    def __init__(self):
        super().__init__('test_mover')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.pub = self.create_publisher(TwistStamped, '/ackermann_like_controller/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = 0.3
        msg.twist.angular.z = 0.0
        self.pub.publish(msg)
        self.get_logger().info(f'publish linear.x=0.3 @ stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}')


def main():
    rclpy.init()
    node = TestMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
