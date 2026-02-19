import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

class HelloworldSubscriber(Node):

  def __init__(self):
    super().__init__('Helloworld_subscriber')
    qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.RELIABLE,
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=10,
      durability=QoSDurabilityPolicy.VOLATILE)
    self.helloworld_subscriber = self.create_subscription(
      String,
      'helloworld',
      self.subscribe_topic_message,
      qos_profile)

  def subscribe_topic_message(self, msg):
    self.get_logger().info('Received message : {0}'.format(msg.data))


def main(args=None):
  rclpy.init(args=args)
  node = HelloworldSubscriber()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt (SIGINT)')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
