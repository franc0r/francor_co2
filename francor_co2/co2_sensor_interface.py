import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class CO2Interface(Node):

    def __init__(self):
        super().__init__('co2_interface')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    co2_interface = CO2Interface()

    rclpy.spin(co2_interface)

    co2_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()