import rclpy
import atexit
import numpy as np
from rclpy.node import Node
from os.path import expanduser
from numpy import linalg as LA
from nav_msgs.msg import Odometry

home = expanduser( '~' )
file = open( './src/waypoints/waypoints_log.csv', 'w' )
file.write( 'x,y,yaw,v\n' )


class WaypointsLogger( Node ):
    """
    Class that defines a ROS2 node to log x, y, yaw angle and speed into a csv file while the car is driving on the track.
    """

    def __init__( self ):
        """
        Constructor for the WaypointsLogger class
        """

        super().__init__( 'waypoint_logger' )

        self.subscription = self.create_subscription( Odometry, 'ego_racecar/odom', self.odomCallback, 10 )

        print( 'Waypoints logger started...' )


    def odomCallback( self, msg ):
        """
        Odometry subscriber callback function. This function extracts the pose and speed of the car from the Odometry message
        and appends a row of x, y, yaw angle and speed to the logging csv file.

        Args:
            msg ( nav_msgs.msg/Odommetry ): Odometry ROS2 message
        """

        # Convert quaternion values to yaw (heading) angle
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w

        siny_cosp = 2 * ( q_w * q_z + q_x * q_y )
        cosy_cosp = 1 - 2 * ( q_y * q_y + q_z * q_z )
        yaw = np.arctan2( siny_cosp, cosy_cosp )

        # Calculate speed
        speed = LA.norm(np.array([msg.twist.twist.linear.x, 
                                msg.twist.twist.linear.y, 
                                msg.twist.twist.linear.z]),2)

        # Write x, y, yaw angle and speed to the csv file
        file.write('%f, %f, %f, %f\n' % (msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        yaw,
                                        speed))
        
        
def shutdown():
    """
    Closes the logging file after logging is stopped using Ctrl+ C
    """
    file.close()
    print( 'Waypoints Logger stopped.' )

    
def main( args = None ):

    atexit.register( shutdown )
    rclpy.init( args = args )
    subscriber = WaypointsLogger()
    rclpy.spin( subscriber )
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()