# ros related
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.timer import Rate
from rclpy.node import Node, Timer, Publisher
from rclpy.action import ActionServer,CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle as GoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# msgs
from std_msgs.msg import Float64MultiArray
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, Pose

# drone
from drone_control.mavsdk_client import MavSDKClient, AttitudeRate, PositionNedYaw
from mpc.drone_mpc_controller import DroneMpcController

# general
import numpy as np
import threading

class thresholds:
    position : float = 0.1
    velocity : float = 0.1
    angular_velocity : float = 0.1
    orientation : float = 0.1

class DroneController(Node):

    mav_sdk_client : MavSDKClient
    controller : DroneMpcController

    # parameters
    control_rate: float
    state_update_rate: float
    thresholds_ : thresholds = thresholds()
    
    # ros related
    controller_rate: Rate
    control_timer: Timer
    state_update_timer: Timer
    state_publisher: Publisher
    position_goal_server: ActionServer
    
    # states & goals
    state: np.ndarray  = np.zeros(13) #TODO: maybe change to state class
    current_goal_state: np.ndarray  = np.zeros(13) #TODO: maybe change to state class
    pose_goals : list[PoseStamped] = []

    # Misc
    state_lock : threading.Lock

    def __init__(self):
        super().__init__('drone_controller_node')

        self.mav_sdk_client = MavSDKClient()
        self.controller = DroneMpcController()
        self.state_lock = threading.Lock()
        self.pose_goals = [PoseStamped()]

        # cb groups
        self.action_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        # parameters
        self.declare_parameter('controller_rate',100.0)
        self.declare_parameter('state_update_rate',200.0)
        self.declare_parameter('threshold.positon',0.1)
        self.declare_parameter('threshold.velocity',0.02)
        self.declare_parameter('threshold.angular_velocity',0.02)
        self.declare_parameter('threshold.orientation',0.1)

        self.control_rate = self.get_parameter('controller_rate').get_parameter_value().double_value
        self.state_update_rate = self.get_parameter('state_update_rate').get_parameter_value().double_value
        self.thresholds_.position = self.get_parameter('threshold.positon').get_parameter_value().double_value
        self.thresholds_.velocity = self.get_parameter('threshold.velocity').get_parameter_value().double_value
        self.thresholds_.angular_velocity = self.get_parameter('threshold.angular_velocity').get_parameter_value().double_value
        self.thresholds_.orientation = self.get_parameter('threshold.orientation').get_parameter_value().double_value

        #TODO: maybe change to joint_states
        self.state_publisher = self.create_publisher(Float64MultiArray, '/drone_state', 10)
        self.controller_rate =  self.create_rate(self.control_rate)
        self.position_goal_server = rclpy.action.ActionServer(
            self, FollowWaypoints, 'drone_set_pose_goal',
            callback_group=self.action_cb_group,
            goal_callback=self.pose_goal_accept_callback,
            cancel_callback=self.pose_goal_cancel_callback,
            execute_callback=self.pose_goal_execute_callback)

    def initialize(self):
        """ Constructor should return fast, so async operations are moved here. """
        # TODO: improve this logic
        mav_sdk_init_future = self.mav_sdk_client.run_async(self.mav_sdk_client.initialize())
        controller_init_future = self.mav_sdk_client.run_async(self.controller.initialize())

        #TODO: handle initialization failures
        mav_sdk_init_future.result() 
        controller_init_future.result() 

        # start timers after connecting to mavsdk
        # self.control_timer = self.create_timer(1.0/self.control_rate, self._control_timer_callback,self.timer_cb_group)
        self.state_update_timer = self.create_timer(1.0/self.state_update_rate, self._state_timer_callback,self.timer_cb_group)
        
    def _state_timer_callback(self):
        """This function publishes the drone system to be available in the ros ecosystem"""
        with self.state_lock:
            # print(self.mav_sdk_client.get_latest_state())
            self.state =  self.mav_sdk_client.get_latest_state()
        msg = Float64MultiArray()
        msg.data = self.state.tolist()
        self.state_publisher.publish(msg)

    def _control_timer_callback(self):
        """Deprecated: This function publishes the drone commands immediately through the mav sdk"""
        pass
        # TODO: just a reference target for now
        # x_ref = np.zeros(13)
        # x_ref[0:3] = np.array([0.0, 0.0, -1.0])
        # x_ref[6:10] = np.array([1.0, 0.0, 0.0, 0.0])

        # attitude_cmd : AttitudeRate  = self.controller.controller_update(self.state,x_ref)
        # self.mav_sdk_client.send_AttitudeRate_cmd(attitude_cmd)       

    def waypoint_reached(self, ):
        with self.state_lock:
            current_state = self.state.copy()
        self.current_goal_state

        pos_error = np.linalg.norm(current_state[0:3] - self.current_goal_state[0:3] )
        vel_error = np.linalg.norm(current_state[3:6] - self.current_goal_state[3:6] )
        orientation_error = 3.0 # TODO fix orientation
        ang_vel_error = np.linalg.norm(current_state[10:13] - self.current_goal_state[10:13] )


        if pos_error > self.thresholds_.position:
            return False
        
        if vel_error > self.thresholds_.velocity:
            return False
        
        # if orientation_error > self.thresholds_.orientation:
            # return False
        
        if ang_vel_error > self.thresholds_.angular_velocity:
            return False
        
        return True 
    
    def hover_at_currrent_position(self):
        with self.state_lock:
            current_state = self.state.copy()
        # TODO: fix
        qw = current_state[6]
        qx = current_state[7]
        qy = current_state[8]
        qz = current_state[9]

        # Numerator and Denominator for atan2
        numerator = 2 * (qw * qz + qx * qy)
        denominator = 1 - 2 * (qy * qy + qz * qz)

        # Calculate yaw in radians
        yaw_rad = np.arctan2(numerator, denominator)

        # Convert to degrees for easier understanding
        yaw_deg = np.degrees(yaw_rad)
        # -----------
        
        pos_cmd = PositionNedYaw(current_state[0],current_state[1],current_state[2],yaw_deg) 

        self.get_logger().info(f'Sending Position Command at North={pos_cmd.north_m:.2}, East={pos_cmd.east_m},Down={pos_cmd.down_m} with Yaw={pos_cmd.yaw_deg}')
        self.mav_sdk_client.send_Position_cmd(pos_cmd)

    def pose_goal_accept_callback(self, goal : FollowWaypoints.Goal ):
        '''
        This function validates the goal target. 
        Due to mutually exlusive, this cannot run when having an active goal (trajectory or position).
        '''
        self.get_logger().info('[Position Goal] Received goal request')
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
        print(goal_poses    )
        print(goal_poses.poses[0]    )
        self.pose_goals[0]  =goal_poses.poses[0]
        print('here')

        self.current_goal_state = np.zeros(13)
        self.current_goal_state[0] = self.pose_goals[0].pose.position.x
        self.current_goal_state[1] = self.pose_goals[0].pose.position.y
        self.current_goal_state[2] = self.pose_goals[0].pose.position.z

        self.current_goal_state[6] = self.pose_goals[0].pose.orientation.w
        self.current_goal_state[7] = self.pose_goals[0].pose.orientation.x
        self.current_goal_state[8] = self.pose_goals[0].pose.orientation.y
        self.current_goal_state[9] = self.pose_goals[0].pose.orientation.z

        print("receveived ")
        print(self.current_goal_state)
        print(goal_handle.is_cancel_requested)

        result = FollowWaypoints.Result() 
        result.missed_waypoints = []
        
        while rclpy.ok() and (not goal_handle.is_cancel_requested) and (not self.waypoint_reached()): #maybe change to cancel requested
            with self.state_lock:
                current_state = self.state.copy()
            attitude_cmd : AttitudeRate  = self.controller.controller_update(current_state,self.current_goal_state)
            self.mav_sdk_client.send_AttitudeRate_cmd(attitude_cmd) 
            
            # using a rate object means if controller update takes < rate, we keep the hz constant, else, we send immediately after this cmd
            # a timer implementation may be better
            self.controller_rate.sleep() 
        self.get_logger().info('[Position Goal] Reached Target')
        self.hover_at_currrent_position()
        self.get_logger().info('[Position Goal] Hovering at Target')

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('[Position Goal] Request cancelled')
            if not self.waypoint_reached():
                result.missed_waypoints = [0] # waypoint id, since it is a list
                return result
                    
        #TODO: check if hovering successful?
        goal_handle.succeed()
        self.get_logger().info('[Position Goal] Goal succeded')
        return result

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