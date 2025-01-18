# filepath: /home/nps/src/tank2/src/motor_driver/motor_driver_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from motor_driver.motor_driver import MotorDriver

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        self.motor_driver = MotorDriver()

        self.publisher_ = self.create_publisher(String, 'motor_status', 10)
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.motor_command_callback,
            10)
        self.subscription  # prevent unused variable warning

    def motor_command_callback(self, msg):
        self.get_logger().info('Received motor command: "%s"' % msg.data)
        if msg.data == 'stop':
            self.motor_driver.stop()
        elif msg.data.startswith('set_tracks'):
            tracks = list(map(int, msg.data.split()[1:]))
            self.motor_driver.set_tracks(tracks)
        elif msg.data.startswith('set_turret'):
            turret = list(map(int, msg.data.split()[1:]))
            self.motor_driver.set_turret(turret)

    def publish_motor_status(self):
        msg = String()
        msg.data = 'Motor status message'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing motor status: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriverNode()
    rclpy.spin(motor_driver_node)
    motor_driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()