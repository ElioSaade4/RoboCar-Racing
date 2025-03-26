import rclpy
from rclpy.node import Node

import numpy as np
import atexit
# import tf
from os.path import expanduser
from numpy import linalg as LA
# from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

home = expanduser('~')
file = open( './src/waypoint_logger/waypoints.csv', 'w' )
file.write( 'x, y, yaw, v\n' )

class MinimalSubscriber(Node):

    def __init__(self):

        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        # quaternion = np.array([msg.pose.pose.orientation.x, 
        #                    msg.pose.pose.orientation.y, 
        #                    msg.pose.pose.orientation.z, 
        #                    msg.pose.pose.orientation.w])
        # euler = tf.transformations.euler_from_quaternion(quaternion)

        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        speed = LA.norm(np.array([msg.twist.twist.linear.x, 
                                msg.twist.twist.linear.y, 
                                msg.twist.twist.linear.z]),2)

        file.write('%f, %f, %f, %f\n' % (msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        yaw,
                                        speed))
        
        
def shutdown():
    file.close()
    print('Goodbye')

    
def main(args=None):

    atexit.register(shutdown)

    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()