import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Header

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('time_example_node')
    time_publisher = node.create_publisher(Header, 'time', 10)
    msg = Header()
    
    rate = node.create_rate(1.0)
    duration = Duration(seconds=1, nanoseconds=0)
    past = node.get_clock().now()
    
    try:
        while rclpy.ok():
            now = node.get_clock().now()
            seconds, nanoseconds = now.seconds_nanoseconds()
            node.get_logger().info('sec {0} nsec {1}'.format(seconds, nanoseconds))
            
            if ((now - past).nanoseconds * 1e-9) > 5:
                node.get_logger().info('Over 5 seconds!')
                past = node.get_clock().now()
        
            msg.stamp = (now + duration).to_msg()
            time_publisher.publish(msg)
            
            rclpy.spin_once(node)
            rate.sleep()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
