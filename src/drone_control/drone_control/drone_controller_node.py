# ros related
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Rate
from rclpy.node import Node, Timer, Publisher, Subscription
from rclpy.action import ActionServer,CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle as GoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import time

# msgs
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose

# drone
from mavsdk.offboard import AttitudeRate, PositionNedYaw
from mpc.drone_mpc_controller import DroneMpcController

# general
import numpy as np
import threading
from typing import Optional, List
from scipy.spatial.transform import Rotation as R, Slerp

class thresholds:
    position : float = 0.1
    velocity : float = 0.1
    angular_velocity : float = 0.1
    orientation : float = 0.1

class DroneController(Node):

    controller : DroneMpcController

    # parameters
    control_rate: float
    thresholds_ : thresholds = thresholds()
    trajectory_max_desired_velocity_linear: float
    trajectory_max_desired_velocity_angular: float
    
    # ros related
    controller_rate: Rate
    state_subscriber: Subscription
    attitude_command_publisher: Publisher
    current_target_publisher: Publisher
    position_goal_server: ActionServer
    trajectory_goal_server: ActionServer
    
    # states & goals
    state: np.ndarray  = np.zeros(13) 
    current_goal_state: np.ndarray  = np.zeros(13) 
    pose_goals : list[PoseStamped] = []
    active_goal : bool = False
    hovering_active: bool = False
    stop_hover = threading.Event() 

    # Misc
    state_lock : threading.Lock
    active_goal_lock: threading.Lock

    def __init__(self):
        super().__init__('drone_controller_node')

        self.controller = DroneMpcController()
        self.active_goal_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.state[6] = 1.0

        # cb groups
        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        # parameters
        self.declare_parameter('state_topic','drone_state_sim')
        self.declare_parameter('attitude_command_topic', '/drone_attitude_cmd')
        self.declare_parameter('position_command_topic', '/drone_position_cmd')
        self.declare_parameter('controller_rate',100.0)
        self.declare_parameter('threshold.positon',0.1)
        self.declare_parameter('threshold.velocity',0.02)
        self.declare_parameter('threshold.angular_velocity',0.02)
        self.declare_parameter('threshold.orientation',0.1)
        self.declare_parameter('trajectory_max_desired_velocity.linear',1)
        self.declare_parameter('trajectory_max_desired_velocity.angular',1)

        state_topic = self.get_parameter('state_topic').get_parameter_value().string_value
        attitude_command_topic = self.get_parameter('attitude_command_topic').get_parameter_value().string_value
        position_command_topic = self.get_parameter('position_command_topic').get_parameter_value().string_value
        self.control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value
        self.thresholds_.position = self.get_parameter('threshold.positon').get_parameter_value().double_value
        self.thresholds_.velocity = self.get_parameter('threshold.velocity').get_parameter_value().double_value
        self.thresholds_.angular_velocity = self.get_parameter('threshold.angular_velocity').get_parameter_value().double_value
        self.thresholds_.orientation = self.get_parameter('threshold.orientation').get_parameter_value().double_value

        #TODO: maybe change to joint_states (Michalis)
        self.state_subscriber = self.create_subscription(Float64MultiArray, state_topic, self.drone_state_cb, 10)
        self.attitude_command_publisher = self.create_publisher(Float64MultiArray,attitude_command_topic, 10)
        self.position_command_publisher = self.create_publisher(Float64MultiArray,position_command_topic, 10)
        self.current_target_publisher = self.create_publisher(PoseStamped, 
                                                              '/drone_current_target', 
                                                              QoSProfile(
                                                                            depth=1, 
                                                                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
                                                                        ))
        self.controller_rate =  self.create_rate(self.control_rate)
        
        self.position_goal_server = rclpy.action.ActionServer(
            self, FollowWaypoints, 'drone_set_pose_goal',
            callback_group=self.action_cb_group,
            goal_callback=self.pose_goal_accept_callback,
            cancel_callback=self.pose_goal_cancel_callback,
            execute_callback=self.pose_goal_execute_callback)
        
        self.trajectory_goal_server = rclpy.action.ActionServer(
            self, FollowWaypoints, 'drone_set_trajectory_goal',
            callback_group=self.action_cb_group,
            goal_callback=self.trajectory_goal_accept_callback,
            cancel_callback=self.trajectory_goal_cancel_callback,
            execute_callback=self.trajectory_goal_execute_callback)

    def drone_state_cb(self, msg: Float64MultiArray):
        with self.state_lock:
            self.state = np.array(msg.data)

    def initialize(self):
        """ Constructor should return fast, so async operations are moved here. """
        self.controller.initialize()
        self.get_logger().info("Controller node initialized")  

    def waypoint_reached(self ):
        with self.state_lock:
            current_state = self.state.copy()
        self.current_goal_state

        pos_error = np.linalg.norm(current_state[0:3] - self.current_goal_state[0:3] )
        vel_error = np.linalg.norm(current_state[3:6] - self.current_goal_state[3:6] )
        orientation_error = 2.0 * np.arccos( np.dot(self.current_goal_state[6:10] , current_state[6:10]) ) 
        ang_vel_error = np.linalg.norm(current_state[10:13] - self.current_goal_state[10:13] )


        if pos_error > self.thresholds_.position:
            return False
        
        if vel_error > self.thresholds_.velocity:
            return False
        
        if orientation_error > self.thresholds_.orientation:
            return False
        
        if ang_vel_error > self.thresholds_.angular_velocity:
            return False
        
        return True 
    
    def ensure_not_hovering(self):
        if self.hovering_active:
            self.stop_hover.set()
            while self.hovering_active:
                time.sleep(5e-4)

            
    def hover_at_currrent_position(self, hover_state : np.ndarray | None = None):
        self.hovering_active = True
        if hover_state is None:
            with self.state_lock:
                hover_state = self.state.copy()

        #1. Get projection in yaw, to stabilize
        qw = hover_state[6]
        qx = hover_state[7]
        qy = hover_state[8]
        qz = hover_state[9]

        # Numerator and Denominator for atan2
        numerator = 2 * (qw * qz + qx * qy)
        denominator = 1 - 2 * (qy * qy + qz * qz)

        # Calculate yaw in radians
        yaw_rad = np.arctan2(numerator, denominator)

        hover_state[6] = np.cos(yaw_rad/2)
        hover_state[7] = 0.0
        hover_state[8] = 0.0
        hover_state[9] = np.sin(yaw_rad/2)
        #2. set velocities to zero:
        hover_state[3:6] = 0.0
        hover_state[10:13] = 0.0
        #3. set current goal state
        current_goal_state = hover_state
        #----------------

        cmd_msg = Float64MultiArray()
        
        while rclpy.ok() and (not self.stop_hover.is_set()):
            with self.state_lock:
                current_state = self.state.copy()
            attitude_cmd : AttitudeRate  = self.controller.controller_update(current_state,current_goal_state)

            cmd_msg.data = [attitude_cmd.roll_deg_s,attitude_cmd.pitch_deg_s,attitude_cmd.yaw_deg_s,attitude_cmd.thrust_value]
            self.attitude_command_publisher.publish(cmd_msg)

            # using a rate object means if controller update takes < rate, we keep the hz constant, else, we send immediately after this cmd
            # a timer implementation may be better
            self.controller_rate.sleep() 
        self.stop_hover.clear()  # reset flag so you can start hovering again later
        self.hovering_active = False


    def pose_goal_accept_callback(self, goal : FollowWaypoints.Goal ):
        '''
        This function validates the goal target. 
        Due to mutually exlusive, this cannot run when having an active goal (trajectory or position).
        '''
        self.get_logger().info('[Position Goal] Received goal request')
        with self.active_goal_lock:
            if self.active_goal:
                self.get_logger().error('[Position Goal] Controller already has an active goal. Rejecting goal')
                return GoalResponse.REJECT
            self.active_goal = True
        goal_poses : FollowWaypoints = goal
        if goal_poses.poses:
            if len(goal_poses.poses) == 1:
                # TODO: Add a heuristic to reject goals that are very far way such that mpc will fail **or** switch to traj execution 
                return GoalResponse.ACCEPT
            else:
                self.get_logger().error('[Position Goal] Multiple waypoints received from the goal interface. Use the trajectory interface instead')
                return GoalResponse.REJECT
        self.get_logger().error('[Position Goal] Empty goal received')
        return GoalResponse.REJECT

    def pose_goal_cancel_callback(self, goal : FollowWaypoints.Goal):
        self.get_logger().info('[Position Goal] Received cancel request')
        return CancelResponse.ACCEPT
    
    def pose_goal_execute_callback(self,goal_handle  : GoalHandle ):
        goal_poses : FollowWaypoints.Goal= goal_handle.request
        self.pose_goals.clear()
        self.pose_goals.append( goal_poses.poses[0] )
        self.current_target_publisher.publish(self.pose_goals[0])

        self.current_goal_state = np.zeros(13)
        self.current_goal_state[0] = self.pose_goals[0].pose.position.x
        self.current_goal_state[1] = self.pose_goals[0].pose.position.y
        self.current_goal_state[2] = self.pose_goals[0].pose.position.z

        self.current_goal_state[6] = self.pose_goals[0].pose.orientation.w
        self.current_goal_state[7] = self.pose_goals[0].pose.orientation.x
        self.current_goal_state[8] = self.pose_goals[0].pose.orientation.y
        self.current_goal_state[9] = self.pose_goals[0].pose.orientation.z

        result = FollowWaypoints.Result() 
        result.missed_waypoints = []
        cmd_msg = Float64MultiArray()
        self.ensure_not_hovering()
        
        while rclpy.ok() and (not goal_handle.is_cancel_requested) and (not self.waypoint_reached()):
            with self.state_lock:
                current_state = self.state.copy()
            attitude_cmd : AttitudeRate  = self.controller.controller_update(current_state,self.current_goal_state)

            cmd_msg.data = [attitude_cmd.roll_deg_s,attitude_cmd.pitch_deg_s,attitude_cmd.yaw_deg_s,attitude_cmd.thrust_value]
            self.attitude_command_publisher.publish(cmd_msg)

            # using a rate object means if controller update takes < rate, we keep the hz constant, else, we send immediately after this cmd
            # a timer implementation may be better
            self.controller_rate.sleep() 
        self.get_logger().info('[Position Goal] Exiting Pose Loop')
        # self.hover_at_currrent_position()
        t = threading.Thread(target=self.hover_at_currrent_position)
        t.start()
        self.get_logger().info('[Position Goal] Hovering at Target')
        
        if goal_handle.is_cancel_requested:
            if not self.waypoint_reached():
                result.missed_waypoints = [0] # waypoint id, since it is a list
                goal_handle.canceled()
                self.get_logger().info('[Trajectory Goal] Request cancelled')
                self.terminate_action()
                return result
                    
        #TODO: check if hovering successful?
        goal_handle.succeed()
        self.get_logger().info('[Position Goal] Goal succeded')
        self.terminate_action()
        return result

    def trajectory_goal_accept_callback(self, goal : FollowWaypoints.Goal ):
        '''
        This function validates the goal target. 
        Due to mutually exlusive, this cannot run when having an active goal (trajectory or position).
        '''
        self.get_logger().info('[Trajectory Goal] Received goal request')
        with self.active_goal_lock:
            if self.active_goal:
                self.get_logger().error('[Trajectory Goal] Controller already has an active goal. Rejecting goal')
                return GoalResponse.REJECT
            goal_poses : FollowWaypoints = goal
            if goal_poses.poses:
                if len(goal_poses.poses) >= 1:
                    self.active_goal = True
                    return GoalResponse.ACCEPT
        self.get_logger().error('[Trajectory Goal] Empty goal received')
        return GoalResponse.REJECT
    
    def trajectory_goal_cancel_callback(self, goal : FollowWaypoints.Goal):
        self.get_logger().info('[Trajectory Goal] Received cancel request')
        return CancelResponse.ACCEPT
    
    def trajectory_goal_execute_callback(self,goal_handle  : GoalHandle ):
        goal_poses : FollowWaypoints.Goal= goal_handle.request

        self.pose_goals.clear()
        num_wp = len(goal_poses.poses )+1
        Xtraj = np.zeros((13,num_wp ))
        Tdef = np.zeros(num_wp)

        for i in range(1, num_wp):
            self.pose_goals.append(goal_poses.poses[i-1])
            self.current_target_publisher.publish(goal_poses.poses[i-1])

            Xtraj[0,i] = goal_poses.poses[i-1].pose.position.x
            Xtraj[1,i] = goal_poses.poses[i-1].pose.position.y
            Xtraj[2,i] = goal_poses.poses[i-1].pose.position.z
            
            Xtraj[6,i] = goal_poses.poses[i-1].pose.orientation.w
            Xtraj[7,i] = goal_poses.poses[i-1].pose.orientation.x
            Xtraj[8,i] = goal_poses.poses[i-1].pose.orientation.y
            Xtraj[9,i] = goal_poses.poses[i-1].pose.orientation.z

            Tdef[i] = float(goal_poses.poses[i-1].header.stamp.sec + 1e-9* goal_poses.poses[i-1].header.stamp.nanosec)

        # add current state at beggining:
        with self.state_lock:
            current_state = self.state.copy()
        Xtraj[:,0] = current_state

        # TODO: we can add a method to interpolate the trajectory to give smoother references (splines etc) 

        self.current_goal_state = Xtraj[:,1]

        result = FollowWaypoints.Result() 
        result.missed_waypoints = []
        cmd_msg = Float64MultiArray()

        self.ensure_not_hovering()
        
        time_expired = 0.0
        while rclpy.ok() and (not goal_handle.is_cancel_requested) and (not self.waypoint_reached()):
            with self.state_lock:
                current_state = self.state.copy()
            traj_desired = self.sample_trajectory(Xtraj,Tdef,current_state,time_expired)
            self.current_goal_state = traj_desired[:,-1] # this is for the waypoint reached. 
            attitude_cmd : AttitudeRate  = self.controller.trajectory_controller_update(current_state,traj_desired)

            cmd_msg.data = [attitude_cmd.roll_deg_s,attitude_cmd.pitch_deg_s,attitude_cmd.yaw_deg_s,attitude_cmd.thrust_value]
            self.attitude_command_publisher.publish(cmd_msg)
            
            # using a rate object means if controller update takes < rate, we keep the hz constant, else, we send immediately after this cmd
            # a timer implementation may be better
            time_expired += 1.0/self.control_rate 
            self.controller_rate.sleep() 
        self.get_logger().info('[Position Goal] Exiting Pose Loop')
        # self.hover_at_currrent_position()
        t = threading.Thread(target=self.hover_at_currrent_position)
        t.start()
        self.get_logger().info('[Trajectory Goal] Hovering at Target')

        if goal_handle.is_cancel_requested:
            if not self.waypoint_reached():
                result.missed_waypoints = [0] #TODO: fix waypoint id, since it is a list
                goal_handle.canceled()
                self.get_logger().info('[Trajectory Goal] Request cancelled')
                self.terminate_action()
                return result
                    
        #TODO: check if hovering successful?
        goal_handle.succeed()
        self.get_logger().info('[Trajectory Goal] Goal succeded')
        self.terminate_action()
        return result
    
    def terminate_action(self):
        with self.active_goal_lock:
            self.active_goal = False

    def sample_trajectory(self, desired_trajectory : np.ndarray, desired_timestamps: List[float],current_state: np.ndarray,  tcurrent: Optional[float] = None):
        """
        This method performs a linear interpolation to give smooth trajectory references.
        It runs in the control loop so it should stay minimal

        This takes <1ms for 50 sampling points
        """

        if tcurrent is None:
            # TODO: [optional] implement feedback in trajectory tracking. Aim: test what is better
            # Idea: Find the closes point to the trajectory (that is why signature has current state) and use its 
            # corresponding timestep (or better interpolated timestep) as the "current time"
            self.get_logger().error("tcurrent was not provided; calculating default value.")
            # tcurrent = ... # [wrong] Example default calculation

        mpc_TH = self.controller.t_H
        mpc_N = self.controller.Nnodes + 1
        mpc_DT = mpc_TH/(mpc_N-1.0)
        T_horizon = tcurrent+mpc_TH
        
        number_of_time_samples = mpc_N
        number_of_constant_samples = 0

        # this handles the interpolation at the end of the trajectory
        if T_horizon > desired_timestamps[-1]:
            number_of_time_samples = int( np.floor ( (desired_timestamps[-1] - tcurrent)/mpc_DT) )+1
            if number_of_time_samples > 1:
                number_of_constant_samples = mpc_N - number_of_time_samples
            else: 
                number_of_constant_samples = mpc_N - 1
            T_horizon = (number_of_time_samples-1)*mpc_DT+tcurrent

        if number_of_time_samples > 1:
        
            time_series_mpc = np.linspace(tcurrent,T_horizon,number_of_time_samples)
            pos_dim = 3
            quat_dim = 4

            pos_interp = np.vstack([np.interp(time_series_mpc, desired_timestamps, desired_trajectory[i,:]) for i in range(pos_dim)])
            quat_interp = DroneController.slerp(desired_trajectory[6:10,:],desired_timestamps,time_series_mpc )

            vel_interp = np.diff(pos_interp, axis=1) / mpc_DT
            vel_interp = np.hstack([vel_interp,vel_interp[:,-1].reshape((3,1))]) #append ending velocity
        else:
            pos_interp = desired_trajectory[:3,-1].reshape(3,1)
            vel_interp = np.zeros((3,1))
            quat_interp = desired_trajectory[6:10,-1].reshape(4,1)
            

        # Stack final target as reference
        if number_of_constant_samples > 0:
            pos_interp  = np.hstack([pos_interp,np.tile(desired_trajectory[:3,-1].reshape(3,1),number_of_constant_samples)])
            vel_interp  = np.hstack([vel_interp,np.zeros((3,number_of_constant_samples))])
            quat_interp = np.hstack([quat_interp,np.tile(desired_trajectory[6:10,-1].reshape(4,1),number_of_constant_samples)])

        ang_vel_interp = np.zeros((3,mpc_N))

        return np.vstack([pos_interp,vel_interp,quat_interp,ang_vel_interp])

    def slerp(desired_trajectory: np.ndarray, desired_timestamps: List[float], query_timestamps: np.ndarray ):
        
        desired_quats = desired_trajectory.T        # convert to (N,4)
        desired_times = desired_timestamps                   # (N,)

        key_rots = R.from_quat(desired_quats)
        slerp = Slerp(desired_times, key_rots)

        interp_rots = slerp(query_timestamps)
        #TODO: estimate angular velocity from rotation matrices

        return  interp_rots.as_quat().T             # return shape (4, M)
        

def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    node.initialize()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()