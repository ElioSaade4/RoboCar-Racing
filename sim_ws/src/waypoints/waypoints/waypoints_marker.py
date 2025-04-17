import rclpy
import pandas as pd
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class WaypointsMarker( Node ):
    """
    Class that defines a ROS2 node that publishes markers of the reference racing line and the racing line of the car running MPC.
    """

    def __init__( self ):
        """
        Constructor for the WaypointsMarker class
        """

        super().__init__( 'waypoints_marker' )
        
        self.line_pub = self.create_publisher( Marker, 'line_marker', 10 )
        
        # Read and parse CSV file for reference waypoints
        df = pd.read_csv( './src/waypoints/waypoints_Levine_2.csv' )
        self.ref_x = df[ 'x' ].to_numpy()
        self.ref_y = df[ 'y' ].to_numpy()
        self.points = self.generateRaceLine( self.ref_x, self.ref_y )

        # Read and parse CSV file for MPC waypoints
        df2 = pd.read_csv( './src/waypoints/waypoints_MPC.csv' )
        self.mpc_x = df2[ 'x' ].to_numpy()
        self.mpc_y = df2[ 'y' ].to_numpy()
        self.mpc_points = self.generateRaceLine( self.mpc_x, self.mpc_y )
        
        # Publish markers periodically
        self.timer1 = self.create_timer( 1, self.publishRaceLine )

        print( 'Waypoints loaded, publishing markers...' )
    
    
    def generateRaceLine( self, x, y ):
        """
        Generates an array of Point objects based on x and y coordinates.

        Args:
            x ( np.array ): array of x coordinates
            y ( np.array ): array of y coordinates

        Returns:
            points ( list ): list of Points objects corresponding the the x and y coordinates
        """

        points = []

        for i in range( len( x ) ):
            point = Point()
            point.x = x[ i ]
            point.y = y[ i ]
            point.z = 0.0 
            points.append(point)

        return points


    def publishRaceLine( self ):
        """
        Publishes 2 ROS2 messages to topic /line_marker. 
        The first message is a marker for the reference raceline. 
        The second message is a marker for the racing line that the car followef using MPC.
        """

        # Marker 1: reference waypoints
        marker1 = Marker()
        marker1.header.frame_id = "map" 
        marker1.header.stamp = self.get_clock().now().to_msg()
        marker1.ns = "line_strip"
        marker1.id = 0
        marker1.type = Marker.LINE_STRIP
        marker1.action = Marker.ADD
        marker1.scale.x = 0.075  # Line width

        # Set color (RGBA)
        marker1.color = ColorRGBA( r = 1.0, g = 0.0, b = 0.0, a = 1.0 )

        # Set lifetime to zero (keeps marker indefinitely)
        marker1.lifetime.sec = 0
        marker1.lifetime.nanosec = 0

        marker1.points = self.points

        self.line_pub.publish( marker1 )

        # Marker 2: MPC trajectory waypoints
        marker2 = Marker()
        marker2.header.frame_id = "map"  
        marker2.header.stamp = self.get_clock().now().to_msg()
        marker2.ns = "line_strip"
        marker2.id = 1
        marker2.type = Marker.LINE_STRIP
        marker2.action = Marker.ADD
        marker2.scale.x = 0.075  # Line width

        # Set color (RGBA)
        marker2.color = ColorRGBA( r = 0.0, g = 0.0, b = 1.0, a = 1.0 )

        # Set lifetime to zero (keeps marker indefinitely)
        marker2.lifetime.sec = 0
        marker2.lifetime.nanosec = 0

        marker2.points = self.mpc_points

        self.line_pub.publish( marker2 )


def main( args = None ):

    rclpy.init( args = args )
    node = WaypointsMarker()
    rclpy.spin( node )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()