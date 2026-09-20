#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, Timer, Publisher, Subscription, Service
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool



import asyncio
import threading
from dataclasses import dataclass, field

from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw
from mavsdk.telemetry import Battery

import time
import numpy as np


@dataclass
class DroneState():
    # State layout published as drone_state_sim: [p(3), v(3), q_xyzw(4), w(3)].
    # q is initialised to the identity rotation in xyzw order so the very first
    # publish (before any telemetry callback fires) carries a valid quaternion.
    p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    q: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    w: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def as_ndarray(self):
        return np.concatenate((self.p,self.v,self.q,self.w))

class MavSDKClient(Node):
    drone: System
    state: DroneState
    state_lock : threading.Lock
    batt_state : Battery

    #states
    armed: bool = False

    #params
    state_update_rate: float = 200.0
    secondary_state_rate: float = 2.0

    #ROS-Related
    state_update_timer: Timer
    secondary_state_timer: Timer
    state_publisher: Publisher
    attitude_cmd_subscriber: Subscription
    pos_cmd_subscriber: Subscription
    takeoff_server: Service
    arming_server: Service

    asyncio_loop : asyncio.BaseEventLoop # start an event loop to avoid conflicts with ros async callbacks
    event_loop_thread : threading.Thread

    def __init__(self):
        super().__init__('mavsdk_client_node')
        self.drone = System()
        self.state = DroneState()
        self.state_lock = threading.Lock()

        # cb groups
        self.secondary_cb_group = MutuallyExclusiveCallbackGroup()
        self.state_timer_cb_group = MutuallyExclusiveCallbackGroup()

        # parameters
        self.declare_parameter('state_update_rate',200.0)
        self.state_update_rate = self.get_parameter('state_update_rate').get_parameter_value().double_value

        #asyncio logic #TODO: verify and discuss
        #start a background thread that manages the asyncio loop, so we asyncronously
        #read states.
        self.asyncio_loop = asyncio.new_event_loop()
        self.event_loop_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.event_loop_thread.start()

    async def initialize(self):
        """
        Initialize Class
        Constructor should return fast, so async operations are moved here. 

        Current implementation returns a future while waiting for connections. So other process can occur while waiting for full connection
        #TODO: handle errors
        """
        await self.drone.connect(system_address="udp://:14540")
        drone_connected = await self._wait_for_connection(timeout_s=15.0)
        if not drone_connected:
            self.get_logger().info("Connection failed")
            return
        drone_armed = await self._wait_until_armable()
        if not drone_armed:
            self.get_logger().info("Vehicle not armable")
            return
        
        self.get_logger().info("[MAV SDK]: Arming...")
        await self.drone.action.arm()
        self.armed = True

        # [REQUIRED]Set initial setpoint before starting offboard
        await self.drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, 0.0))
        
        self.get_logger().info("[MAV SDK]: Starting offboard...")
        await self.drone.offboard.start() 

        self.start_telemetry()

        self.state_update_timer = self.create_timer(1.0/self.state_update_rate, self._state_timer_callback,self.state_timer_cb_group)
        self.secondary_state_timer = self.create_timer(1.0/self.secondary_state_rate, self._secondary_timer_callback,self.secondary_cb_group)

        #TODO: handle failures
        #ros related (need to be initialized after drone connection)
        self.state_publisher = self.create_publisher(Float64MultiArray, 'drone_state_sim', 10)
        self.attitude_cmd_subscriber = self.create_subscription(Float64MultiArray, 'drone_attitude_cmd',self.attitudeRate_cmd_cb,10)
        self.pos_cmd_subscriber = self.create_subscription(Float64MultiArray, 'drone_position_cmd',self.position_cmd_cb,10)
        
        self.arming_server = self.create_service(SetBool, '/arm_drone', self.arm_drone_cb,callback_group=self.secondary_cb_group)
        self.takeoff_server = self.create_service(SetBool, '/takeoff_drone', self.takeoff_drone_cb,callback_group=self.secondary_cb_group)
        
        self._logger.info("Initialized")

    def _secondary_timer_callback(self):
        """This function publishes the secondary and debugging info for the gui"""
        pass
        #1. Battery
        # batt_v = float(self.batt_state.voltage_v or 0.0)
        # batt_pct = float(self.batt_state.remaining_percent * 100.0) if self.batt_state.remaining_percent is not None else 0.0
        #TODO: add a publisher, (Hassan)



    def _state_timer_callback(self):
        """This function publishes the drone system to be available in the ros ecosystem"""
        # temp : np.ndarray =  self.get_latest_state_safe()
        msg = Float64MultiArray()
        msg.data = self.get_latest_state_safe().tolist()
        self.state_publisher.publish(msg)
        
    def attitudeRate_cmd_cb(self,msg :Float64MultiArray ):
        if len(msg.data) == 4:
            roll_r  = msg.data[0]
            pitch_r = msg.data[1]
            yaw_r   = msg.data[2]
            thrust  = msg.data[3]
            self.send_AttitudeRate_cmd(AttitudeRate(roll_r,pitch_r,yaw_r,thrust))

    def send_AttitudeRate_cmd(self, cmd : AttitudeRate):
        #TODO: verify that it is best way (Michalis)
        self.run_async(self.drone.offboard.set_attitude_rate(cmd))

    def position_cmd_cb(self,msg :Float64MultiArray ):
        if len(msg.data) == 4:
            north_m  = msg.data[0]
            east_m   = msg.data[1]
            down_m   = msg.data[2]
            yaw_deg  = msg.data[3]
            self.send_Position_cmd(PositionNedYaw(north_m,east_m,down_m,yaw_deg))

    def send_Position_cmd(self, cmd : PositionNedYaw):
        self.run_async(self.drone.offboard.set_position_ned(cmd))

    def run_async(self, coroutine):
        """ 
        Convenience function that schedule async coroutines safely from another thread.
        It is usefull, since in current implementation, we launch a thread that is responsible
        for the event loop of asyncio

        USE: ros callbacks should use this if interacting with mavsdk, since they cannot directly await.
        """
        return asyncio.run_coroutine_threadsafe(coroutine, self.asyncio_loop)
    
    def _run_event_loop(self):
        """
        Function for running the asyncio event loop
        """
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_forever()
    
    def get_latest_state_safe(self):
        """
        get latest state in a safe manner.
        """
        with self.state_lock:
            return self.state.as_ndarray()
    
    def start_telemetry(self):
        """Function that start the telemetry streams with mavsdk"""

        self.run_async(self._position_listener())
        self.run_async(self._attitude_listener())
        self.run_async(self._odometry_listener())
        self.run_async(self._battery_listener())

    async def _battery_listener(self):
        async for bat in self.drone.telemetry.battery():
            self.batt_state = bat

    async def _position_listener(self):
        """Expressed in Earth Frame NED frame"""
        async for pv in self.drone.telemetry.position_velocity_ned():
            p = pv.position
            v = pv.velocity
            with self.state_lock:
                self.state.p = np.array([p.north_m, p.east_m, p.down_m])
                self.state.v = np.array([v.north_m_s, v.east_m_s, v.down_m_s])

    async def _attitude_listener(self):
        """Expressed in Earth Frame NED frame"""
        async for att in self.drone.telemetry.attitude_quaternion():
            with self.state_lock:
                # xyzw order — matches drone_state_sim layout consumed by
                # state_bridge_node and rl/hovering_experiment.
                self.state.q = np.array([att.x, att.y, att.z, att.w])

    async def _odometry_listener(self):
        """Expresses in Body NED frame (roll, pitch, yaw)"""
        async for odo in self.drone.telemetry.odometry():
            with self.state_lock:
                self.state.w = np.array([odo.angular_velocity_body.roll_rad_s,
                                        odo.angular_velocity_body.pitch_rad_s,
                                         odo.angular_velocity_body.yaw_rad_s])
       
    def takeoff_drone_cb(self, request: SetBool.Request, response: SetBool.Response):
        req = request.data
        if (not self.armed) and (req):
            self.get_logger().error('Currently in disarmed state. Cannot takeoff')
            response.success = False
            response.message = 'Currently in disarmed state. Cannot takeoff'
            return response

        try:
            if req:
                future = self.run_async(self.takeoff())
                future.result(timeout=10.0)
                response.success = True
                response.message = 'Successful Takeoff'
                self.get_logger().info('Successful Takeoff')
            else:
                future = self.run_async(self.land())
                future.result(timeout=10.0)
                response.success = True
                response.message = 'Successful Landing'
                self.get_logger().info('Successful Landing')
        except Exception as e:
            response.success = False
            if req:
                response.message = f'Take-off failed: {str(e)}'
            else:
                response.message = f'Landing failed: {str(e)}'

        return response
  
    async def takeoff(self, altitude_m: float = 1.0):
        """Takeoff to specified altitude (positive value)."""
        self.get_logger().info(f"[MAV SDK]: Taking off to {altitude_m}m...")
        await self.drone.action.set_takeoff_altitude(altitude_m)
        await self.drone.action.takeoff()
    
    async def land(self):
        """Land the drone."""
        self.get_logger().info("[MAV SDK]: Landing...")
        await self.drone.action.land()
    
    def arm_drone_cb(self, request: SetBool.Request, response: SetBool.Response):
        req = request.data
        try:
            if req:
                future = self.run_async(self.arm())
                future.result(timeout=10.0)
                response.success = True
                response.message = 'Arm command sent successfully'
                self.armed = True
                self.get_logger().info('Drone armed')
            else:
                future = self.run_async(self.disarm())
                future.result(timeout=5.0)
                response.success = True
                response.message = 'Disarm command sent successfully'
                self.armed = False
                self.get_logger().info('Drone disarmed')
        except Exception as e:
            response.success = False
            if req:
                response.message = f'Arming failed: {str(e)}'
            else:
                response.message = f'Disarming failed: {str(e)}'

        return response

    async def disarm(self):
        """Disarm the drone."""
        self.get_logger().info("[MAV SDK]: Disarming...")
        await self.drone.action.disarm()
    
    async def arm(self):
        """Arm the drone."""
        self.get_logger().info("[MAV SDK]: Arming...")
        await self.drone.action.arm()

    async def _wait_for_connection(self,timeout_s: float = 10.0):
        """Wait for MAVSDK connection."""
        self.get_logger().info("[MAV SDK]: Waiting connection...")
        t0 = time.time()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Connected to vehicle")
                return True
            if time.time() - t0 > timeout_s:
                self.get_logger().info("Connection timeout")
                return False
            await asyncio.sleep(0.1)

    async def _wait_until_armable(self, timeout_s: float = 30.0):
        """Wait until vehicle health indicates armable state."""
        self.get_logger().info("[MAV SDK]: Waiting for vehicle to be armable...")
        t0 = time.time()
        async for health in self.drone.telemetry.health():
            self.get_logger().info(f"Health: global={health.is_global_position_ok}, "
                f"local={health.is_local_position_ok}, "
                f"is_armable={health.is_armable}")
            if health.is_armable:
                return True
            if time.time() - t0 > timeout_s:
                self.get_logger().info("Timed out waiting for armable")
                return False
            await asyncio.sleep(0.5)
        return False
    
    
    
     # TODO: disarm


def main(args=None):
    rclpy.init(args=args)

    mav_sdk_client = MavSDKClient()
    init_future = mav_sdk_client.run_async(mav_sdk_client.initialize())
    init_future.result() # wait for initialization

    executor = MultiThreadedExecutor()
    executor.add_node(mav_sdk_client)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        mav_sdk_client.destroy_node()
        rclpy.shutdown()