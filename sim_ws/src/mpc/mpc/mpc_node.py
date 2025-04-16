import math
import numpy as np
import pandas as pd
from dataclasses import dataclass, field

import cvxpy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped

from std_msgs.msg import Float32


@dataclass
class MPCConfig:
    """
    Dataclass that includes all the configuration parameters for the MPC
    """
    NXK: int = 4    # length of state vector: z = [x, y, v, yaw]
    NU: int = 2     # length of control input vector: u =  [steering angle, acceleration]
    TK: int = 8     # length of finite horizon

    # Cost function matrices
    R: list = field(
        default_factory = lambda: np.diag( [ 5.0, 85.0 ] ) 
    )  # input cost matrix, penalty for inputs - [accel, steering]
    Rd: list = field(
        default_factory = lambda: np.diag( [ 5.0, 85.0 ] ) # maybe 100.0 for steering
    )  # input difference cost matrix, penalty for change of inputs - [accel, steering]
    Q: list = field(
        default_factory = lambda: np.diag( [ 20, 20, 5.5, 20 ] )
    )  # state error cost matrix
    Qf: list = field(
        default_factory = lambda: np.diag( [ 20, 20, 5.5, 20 ] )
    )  # final state error matrix, penalty

    DT: float = 0.1            # time step [s] kinematic
    dlk: float = 0.03           # dist step [m] kinematic
    LENGTH: float = 0.58        # Length of the vehicle [m]
    WIDTH: float = 0.31         # Width of the vehicle [m]
    WB: float = 0.33            # Wheelbase [m]
    MIN_STEER: float = -0.4189  # maximum steering angle [rad]
    MAX_STEER: float = 0.4189   # maximum steering angle [rad]
    MAX_DSTEER: float = np.deg2rad( 180.0 )  # maximum steering speed [rad/s]
    MAX_SPEED: float = 6.0  # maximum speed [m/s]
    MIN_SPEED: float = 0.0  # minimum backward speed [m/s]
    MAX_ACCEL: float = 3.0  # maximum acceleration [m/ss]


class MPC( Node ):

    def __init__( self ):
        """
        Constructor for the MPC class
        """

        super().__init__( 'mpc_node' )

        # Create ROS subscribers and publishers
        self.odom_subscriber = self.create_subscription( Odometry, 
                                                        'ego_racecar/odom',
                                                        self.poseCallback,
                                                        10 )
        
        self.drive_publisher = self.create_publisher( AckermannDriveStamped, '/drive', 10 )

        self.ref_pub = self.create_publisher( Marker, 'ref_markers', 10 )
        self.mpc_pub = self.create_publisher( Marker, 'mpc_markers', 10 )
        self.pose_pub = self.create_publisher( Marker, 'pose_markers', 10 )

        self.dt_pub = self.create_publisher( Float32, 'delta_t', 10 )

        # Load waypoints from csv file
        df = pd.read_csv( './src/waypoints/newwaypoints.csv' )
        self.waypoints_x = df[ 'x' ]
        self.waypoints_y = df[ 'y' ]
        self.waypoints_v = df[ 'v' ]
        self.waypoints_yaw = df[ 'yaw' ]
        
        # State vector z = [x, y, v, yaw]
        self.state = [ 0.0, 0.0, 0.0, 0.0 ]

        # Variables to store steering and acceleration from MPC
        self.mpc_steer = None     
        self.mpc_a = None

        self.steer_cmd = 0.0

        # Initialize MPC problem
        self.config = MPCConfig()
        self.mpcProblemInit()

        # Start timer for MPC controller
        self.timer1 = self.create_timer( self.config.DT, self.mpcCallback )   

        self.prev_time = self.get_clock().now()


    def poseCallback( self, odom_msg ):
        """
        Callback function for the Odometry topic subscriber. 
        Extracts position, orientation and speed from the Odometry message.

        Args:
            odom_msg (nav_msgs/msg/Odometry): odometry message
        """

        # Extract pose from ROS pose msg  
        # x = odom_msg.pose.pose.position.x
        # y = odom_msg.pose.pose.position.y
        v = np.linalg.norm(np.array( [ odom_msg.twist.twist.linear.x, 
                                       odom_msg.twist.twist.linear.y, 
                                       odom_msg.twist.twist.linear.z ] ), 2 )
        
        q_x = odom_msg.pose.pose.orientation.x      # Quaternion x
        q_y = odom_msg.pose.pose.orientation.y      # Quaternion y
        q_z = odom_msg.pose.pose.orientation.z      # Quaternion z
        q_w = odom_msg.pose.pose.orientation.w      # Quaternion w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = np.arctan2( siny_cosp, cosy_cosp )    # Yaw angle from quaternion

        # Transform from rear axle of the car to CoG
        x = odom_msg.pose.pose.position.x + ( 0.165 * math.cos( yaw ) )
        y = odom_msg.pose.pose.position.y + ( 0.165 * math.sin( yaw ) )

        # Update state vector
        self.state = [ x, y, v, yaw ]

    
    def mpcCallback( self ):
        """
        Callback function for the MPC timer that runs at a period of 0.1s.
        This function solves the MPC problem at every iteration and published the steering and speed commands.
        """

        # Publish time difference between calls to mpcCallback(). Used to monitor "real-time" of ROS timer
        end_time = self.get_clock().now()
        time_diff = end_time - self.prev_time
        dt_msg = Float32()
        dt_msg.data = time_diff.nanoseconds / 1e6
        self.dt_pub.publish( dt_msg )
        self.prev_time = end_time  

        # Calculate the reference states for the finite horizon
        ref_states = self.calcRefStates()
        x0 = self.state

        # Solve MPC control problem
        self.mpc_a, self.mpc_steer, mpc_x, mpc_y, _, _, = self.mpcSolve( ref_states, x0 )

        # Publish messages to display points
        self.displayRefPath( ref_states )
        self.displayPose( x0[0], x0[1] )
        self.displayPathPredict( mpc_x, mpc_y )

        # Publish drive message
        self.steer_cmd = self.mpc_steer[0]
        speed_cmd = self.state[ 2 ] + ( self.mpc_a[ 0 ] * self.config.DT )

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "drive_frame" 
        drive_msg.drive.speed = speed_cmd
        drive_msg.drive.steering_angle = self.steer_cmd 
        drive_msg.drive.acceleration = self.mpc_a[ 0 ]
        self.drive_publisher.publish( drive_msg )

        
    def mpcProblemInit(self):
        """
        Sets up the optimization problem in function of Variables and Parameters.
        Variables are the values that the optimization solves for: control inputs and the predicted states
        Parameters are inputs to the optimization problem but that can change between runs: 
        initial state, reference trajectory, linearization matrices.

        The initialization of the MPC problem is done one, then it can be solved at every iteration with
        the option of changing the Parameters.
        """
        
        # Variable for predicted states
        self.xk = cvxpy.Variable( ( self.config.NXK, self.config.TK + 1) )
        
        # Variable for control inputs
        self.uk = cvxpy.Variable( (self.config.NU, self.config.TK) )

        # Initialize reference vectors
        self.x0k = cvxpy.Parameter( ( self.config.NXK, ) )
        self.x0k.value = np.zeros( (self.config.NXK, ) )

        # Initialize reference trajectory parameter
        self.ref_traj_k = cvxpy.Parameter( ( self.config.NXK, self.config.TK + 1 ) )
        self.ref_traj_k.value = np.zeros( ( self.config.NXK, self.config.TK + 1 ) )

        objective = 0.0     # Objective value of the optimization problem
        constraints = []    # Constraints array

        # Objective part 1: deviation of state from reference trajectory weighted by Q, except final time step
        for k in range( self.config.TK ):
            objective += cvxpy.quad_form( self.xk[:,k] - self.ref_traj_k[:,k], self.config.Q )
        
        # Objective part 2: deviation of final state from reference trajectory weighted by Qf
        objective += cvxpy.quad_form( self.xk[:, self.config.TK ] - self.ref_traj_k[ :,self.config.TK ], self.config.Qf )

        # Objective part 3: Influence of the control inputs: Inputs u weighted by R
        for k in range( self.config.TK ):
            objective += cvxpy.quad_form( self.uk[ :, k ], self.config.R )

        # Objective part 4: Penalty on change in control input weighted by Rd (for smoother control commands)
        for k in range( self.config.TK - 1 ):
            objective += cvxpy.quad_form( self.uk[ :, k + 1 ] - self.uk[ :, k ], self.config.Rd )

        # Intialize parameters to stores linearized model matrices A, B, C for every time step
        self.Ak = cvxpy.Parameter( ( self.config.NXK, self.config.TK * self.config.NXK ) ) 
        self.Ak.value = np.zeros( ( self.config.NXK, self.config.TK * self.config.NXK ) )

        self.Bk = cvxpy.Parameter( ( self.config.NXK, self.config.TK * self.config.NU ) )
        self.Bk.value = np.zeros( ( self.config.NXK, self.config.TK * self.config.NU ) )

        self.Ck = cvxpy.Parameter( ( self.config.NXK, self.config.TK ) )
        self.Ck.value = np.zeros( ( self.config.NXK, self.config.TK ) )

        # Initial state constraint
        constraints = [ self.xk[ :, 0 ] == self.x0k ]

        # Linearized state equation constraint
        for k in range( self.config.TK ):
            constraints += [ self.xk[ :,k+1] == self.Ak[ :, k*4:(k+1)*4 ] @ self.xk[:,k] + self.Bk[ :, k*2:(k+1)*2 ] @ self.uk[:,k] + self.Ck[ :, k ] ]

        # Constraints for upper and lower bounds of states and inputs
        x_min = np.array( [ -17, -2, self.config.MIN_SPEED, -math.pi ] )
        x_max = np.array( [ 11, 10, self.config.MAX_SPEED, math.pi ] )
        u_min = np.array( [ -self.config.MAX_ACCEL, self.config.MIN_STEER ] )
        u_max = np.array( [ self.config.MAX_ACCEL, self.config.MAX_STEER ] )
        
        for k in range( self.config.TK ):
            constraints += [ x_min <= self.xk[:,k], self.xk[:,k] <= x_max]
            constraints += [u_min <= self.uk[:,k], self.uk[:,k] <= u_max]

        # Create minimization optimization problem 
        self.MPC_prob = cvxpy.Problem( cvxpy.Minimize( objective ), constraints )


    def calcRefStates( self ):
        """
        Calculates the reference states for the finite horizon given the current state of the vehicle and the reference waypoints.

        Returns:
            ref_traj: 2-D array of reference states for the finite horizon
        """
        # Create placeholder Arrays for the reference trajectory for T steps
        ref_states = np.zeros( ( self.config.NXK, self.config.TK + 1 ) )

        ncourse = len( self.waypoints_x )

        # Find nearest index/setpoint from where the trajectories are calculated
        # _, _, _, ind = nearest_point(np.array([state.x, state.y]), np.array([cx, cy]).T)
        d = np.sqrt( ( self.waypoints_x - self.state[ 0 ] ) ** 2 + ( self.waypoints_y - self.state[ 1 ] ) ** 2 )
        ind = np.argmin( d )

        # Load the initial parameters from the setpoint into the trajectory
        # MIGHT NOT NEED THIS
        ref_states[ 0, 0 ] = self.waypoints_x[ ind ]
        ref_states[ 1, 0 ] = self.waypoints_y[ ind ]
        ref_states[ 2, 0 ] = self.waypoints_v[ ind ]
        ref_states[ 3, 0 ] = self.waypoints_yaw[ ind ] 

        # based on current velocity, distance traveled on the ref line between time steps
        travel = abs( self.state[ 2 ] ) * self.config.DT   # distance travelled per time step 0.1s
        dind = travel / self.config.dlk         # equally space distance points in waypoints.csv

        ind_list = int( ind ) + np.insert(            # get indices that moves the car travel distance
        np.cumsum(np.repeat(1, self.config.TK)), 0, 0
        ).astype(int)

        # ind_list = int(ind) + np.insert(            # get indices that moves the car travel distance
        #     np.cumsum(np.repeat(dind, self.config.TK)), 0, 0
        # ).astype(int)
        
        # correct out of bounds indices (finish line points)
        ind_list[ ind_list >= ncourse ] -= ncourse  

        ref_states[ 0, : ] = self.waypoints_x[ ind_list ]
        ref_states[ 1, : ] = self.waypoints_y[ ind_list ]
        ref_states[ 2, : ] = self.waypoints_v[ ind_list ]

        # Handle transition between 3.14 and -3.14 rad yaw angle
        yaw = self.waypoints_yaw 
        yaw[ self.state[ 3 ] - yaw > 4.7 ] = yaw[ self.state[ 3 ] - yaw > 4.7 ] + ( 2 * math.pi )
        yaw[ self.state[ 3 ]- yaw < -4.7 ] = yaw[ self.state[ 3 ] - yaw < -4.7 ] - ( 2 * math.pi )

        ref_states[ 3, : ] = yaw[ ind_list ]

        return ref_states


    def getModelMatrices( self, v, phi, delta ):
        """
        Calculates the linearized model matrices A, B and C evaluated around a given speed, heading and steering angle.

        Args:
            v ( float ): speed (m/s)
            phi ( float ): heading angle (rad)
            delta ( float ): steering angle (rad)

        Returns:
            A: A matrix of the linearized model evaluated around v, phi and delta
            B: B matrix of the linearized model evaluated around v, phi and delta
            C: C matrix of the linearized model evaluated around v, phi and delta
        """

        # State (or system) matrix A, 4x4
        A = np.zeros( ( self.config.NXK, self.config.NXK ) )
        A[ 0, 0 ] = 1.0
        A[ 1, 1 ] = 1.0
        A[ 2, 2 ] = 1.0
        A[ 3, 3 ] = 1.0
        A[ 0, 2 ] = self.config.DT * math.cos( phi )
        A[ 0, 3 ] = -self.config.DT * v * math.sin( phi )
        A[ 1, 2 ] = self.config.DT * math.sin( phi )
        A[ 1, 3 ] = self.config.DT * v * math.cos( phi )
        A[ 3, 2 ] = self.config.DT * math.tan( delta ) / self.config.WB

        # Input Matrix B; 4x2
        B = np.zeros( ( self.config.NXK, self.config.NU ) )
        B[ 2, 0 ] = self.config.DT
        B[ 3, 1 ] = self.config.DT * v / ( self.config.WB * math.cos( delta ) ** 2 )

        C = np.zeros( ( self.config.NXK, 1 ) )
        C[ 0 ] = self.config.DT * v  * phi * math.sin( phi )
        C[ 1 ] = -self.config.DT * v * phi * math.cos( phi )
        C[ 3 ] = -self.config.DT * v * delta / ( self.config.WB * math.cos( delta ) ** 2 )

        return A, B, C


    def mpcSolve( self, ref_traj, x0 ):
        
        self.x0k.value = x0

        Ak = np.zeros( ( self.config.NXK, self.config.TK * self.config.NXK ) )
        Bk = np.zeros( ( self.config.NXK, self.config.TK * self.config.NU ) )
        Ck = np.zeros( ( self.config.NXK, self.config.TK ) )

        for k in range( self.config.TK ):
            A, B, C = self.getModelMatrices( ref_traj[ 2, k ], ref_traj[ 3, k ], self.steer_cmd ) 
            Ak[ :, k*4:(k+1)*4 ] = A
            Bk[ :, k*2:(k+1)*2 ] = B
            Ck[ :, k ] = C.T

        self.Ak.value = Ak
        self.Bk.value = Bk
        self.Ck.value = Ck

        self.ref_traj_k.value = ref_traj

        # Solve the optimization problem with OSQP solver
        self.MPC_prob.solve(solver=cvxpy.OSQP, verbose=False, warm_start=True)

        # Check if optimization problem has a solution
        if (
            self.MPC_prob.status == cvxpy.OPTIMAL
            or self.MPC_prob.status == cvxpy.OPTIMAL_INACCURATE
        ):
            mpc_x = np.array( self.xk.value[ 0, : ] ).flatten()
            mpc_y = np.array( self.xk.value[ 1, : ] ).flatten()
            mpc_v = np.array( self.xk.value[ 2, : ] ).flatten()
            mpc_yaw = np.array( self.xk.value[ 3, : ] ).flatten()
            mpc_a = np.array( self.uk.value[ 0, : ] ).flatten()
            mpc_steer = np.array( self.uk.value[ 1, : ] ).flatten()

        else:
            print( "Error: Cannot solve mpc.." )
            mpc_a, mpc_steer, mpc_x, mpc_y, mpc_v, mpc_yaw = None, None, None, None, None, None

        return mpc_a, mpc_steer, mpc_x, mpc_y, mpc_v, mpc_yaw


    def displayRefPath( self, ref_path ):

        marker = Marker()
        marker.header.frame_id = "map"  # Change frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spherelist"
        marker.id = 1
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        points = []
        for i in range( self.config.TK ):
            point = Point()
            point.x = ref_path[ 0, i ]
            point.y = ref_path[ 1, i ]
            point.z = 0.0  # Set z = 0 for 2D
            points.append(point)

        marker.points = points 

        marker.scale.x = 0.15  
        marker.scale.y = 0.15 
        marker.scale.z = 0.15 

        # Set color (RGBA)
        marker.color = ColorRGBA( r = 1.0, g = 0.0, b = 0.0, a = 1.0 )

        # Set lifetime to zero (keeps marker indefinitely)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.ref_pub.publish(marker)


    def displayPathPredict( self, mpc_x, mpc_y ):

        marker = Marker()
        marker.header.frame_id = "map"  # Change frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spherelist"
        marker.id = 2
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        points = []
        for i in range( self.config.TK ):
            point = Point()
            point.x = mpc_x[ i ]
            point.y = mpc_y[ i ]
            point.z = 0.0  # Set z = 0 for 2D
            points.append(point)

        marker.points = points 

        marker.scale.x = 0.15  
        marker.scale.y = 0.15
        marker.scale.z = 0.15 

        # Set color (RGBA)
        marker.color = ColorRGBA( r = 0.0, g = 1.0, b = 0.0, a = 1.0 )

        # Set lifetime to zero (keeps marker indefinitely)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.mpc_pub.publish(marker)


    def displayPose( self, x, y ):

        marker = Marker()
        marker.header.frame_id = "map"  # Change frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sphere"
        marker.id = 3
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        marker.scale.x = 0.15 
        marker.scale.y = 0.15 
        marker.scale.z = 0.15

        # Set color (RGBA)
        marker.color = ColorRGBA( r = 0.0, g = 1.0, b = 1.0, a = 1.0 )

        # Set lifetime to zero (keeps marker indefinitely)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.pose_pub.publish(marker)


def main( args = None ):
    rclpy.init( args = args )
    print( "MPC Initialized" )
    mpc_node = MPC()
    rclpy.spin( mpc_node )

    mpc_node.destroy_node()
    rclpy.shutdown()
