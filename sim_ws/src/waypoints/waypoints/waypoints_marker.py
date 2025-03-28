import rclpy
import numpy as np
import pandas as pd
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry


class WaypointsMarker(Node):

    def __init__(self):

        super().__init__('waypoints_marker')
        
        self.line_pub = self.create_publisher( Marker, 'line_marker', 10 )
        self.point_pub = self.create_publisher( Marker, 'waypoint_marker', 10 )

        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.updateOdom,
            10)
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        
        # Read and parse CSV file
        df = pd.read_csv( './src/waypoints/waypoints.csv' )
        self.x = df[ 'x' ].to_numpy()
        self.y = df[ ' y' ].to_numpy()
        self.points = self.generateRaceLine()
        
        # Publish marker periodically
        self.timer1 = self.create_timer( 1, self.publishRaceLine )
        self.timer2 = self.create_timer( 0.2, self.publishWaypoint )

        print( 'Waypoints loaded, publishing...' )
    

    def updateOdom( self, msg ):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
    
    
    def generateRaceLine(self):

        points = []

        for i in range( len( self.x ) ):
            point = Point()
            point.x = self.x[ i ]
            point.y = self.y[ i ]
            point.z = 0.0  # Set z = 0 for 2D
            points.append(point)

        return points


    def publishRaceLine(self):

        marker = Marker()
        marker.header.frame_id = "map"  # Change frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "line_strip"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.075  # Line width

        # Set color (RGBA)
        marker.color = ColorRGBA(r=0.1176, g=0.6471, b=0.7255, a=1.0)

        # Set lifetime to zero (keeps marker indefinitely)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        marker.points = self.points

        self.line_pub.publish(marker)

    
    def publishWaypoint( self ):

        d = np.sqrt( ( self.x - self.pose_x ) ** 2 + ( self.y - self.pose_y ) ** 2 )
        i = np.argmin( d )

        while True:
            i = ( i + 1 ) % 253

            if d[ i ] > 2:
                i = ( i + 253 ) % 253
                break

        marker = Marker()
        marker.header.frame_id = "map"  # Change frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sphere"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.x[i]
        marker.pose.position.y = self.y[i]
        marker.pose.position.z = 0.0

        marker.scale.x = 0.25  
        marker.scale.y = 0.25 
        marker.scale.z = 0.25 

        # Set color (RGBA)
        marker.color = ColorRGBA( r = 1.0, g = 0.0, b = 0.0, a = 1.0 )

        # Set lifetime to zero (keeps marker indefinitely)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.point_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)
    node = WaypointsMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()