import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/stretch/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def test_print(self):
        joint_state = self.joint_state
        self.get_logger().info('In function: {0}'.format(self.joint_state.name[0]))

    def listener_callback(self, joint_state):
        self.joint_state = joint_state
        self.get_logger().info('In callback: {0}'.format(self.joint_state.name[0]))

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    while rclpy.ok():
            rclpy.spin_once(minimal_subscriber)
            minimal_subscriber.test_print()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()