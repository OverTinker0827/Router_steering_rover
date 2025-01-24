import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
import threading
import math
class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/odom/filtered', 10)
        self.running = True
        
        # Start a separate thread for user input
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.start()

    def get_user_input(self):
        while self.running:
            try:
                euler_z = float(input("Enter Euler Z angle in degrees: "))
                quaternion = quaternion_from_euler(0.0, 0.0, math.radians(euler_z))
                
                # Create and publish the Odometry message
                msg = Odometry()
                msg.pose.pose.position.x = 0.0
                msg.pose.pose.position.y = 0.0
                msg.pose.pose.orientation.x = quaternion[0]
                msg.pose.pose.orientation.y = quaternion[1]
                msg.pose.pose.orientation.z = quaternion[2]
                msg.pose.pose.orientation.w = quaternion[3]
                
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published Odometry with Euler Z: {euler_z} degrees')
            except ValueError:
                self.get_logger().warn("Invalid input. Please enter a valid number.")

    def destroy_node(self):
        self.running = False
        self.input_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()