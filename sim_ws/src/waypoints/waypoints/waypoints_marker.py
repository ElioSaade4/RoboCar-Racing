import rclpy
import csv
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
# import pandas as pd

class WaypointsMarker(Node):

    def __init__(self):

        super().__init__('waypoints_marker')
        
        # Declare parameter for CSV file
        self.declare_parameter('csv_file', './src/waypoints/waypoints.csv')
        csv_file_path = self.get_parameter('csv_file').value
        
        self.line_pub = self.create_publisher( Marker, 'line_marker', 10 )
        self.point_pub = self.create_publisher( Marker, 'waypoint_marker', 10 )
        
        # Read and parse CSV file
        self.points = self.read_csv( csv_file_path )
        
        # Publish marker periodically
        self.timer = self.create_timer( 0.2, self.publishMarkers )

        print( 'Waypoints loaded, publishing...' )
    

    def read_csv(self, file_path):
        points = []
        try:
            with open(file_path, 'r') as file:
                reader = csv.reader(file)
                next(reader)  # Skip header if exists
                for row in reader:
                    x, y = map(float, row[ 0:2 ] )
                    point = Point()
                    point.x, point.y, point.z = x, y, 0.0  # Set z = 0 for 2D
                    points.append(point)
        except Exception as e:
            self.get_logger().error(f"Error reading CSV file: {e}")
        return points


    def publishLine(self):

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

    
    def publishPoint( self ):

        marker = Marker()
        marker.header.frame_id = "map"  # Change frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sphere"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
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

    
    def publishMarkers( self ):

        self.publishLine()
        self.publishPoint()



def main(args=None):

    rclpy.init(args=args)
    node = WaypointsMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()