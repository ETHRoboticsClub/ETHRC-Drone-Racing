#!/usr/bin/env python3
import asyncio
import threading
from dataclasses import dataclass

from mavsdk import System
from mavsdk.offboard import AttitudeRate, PositionNedYaw

import time
import numpy as np


@dataclass
class DroneState():
    p: np.ndarray = np.zeros(3)
    v: np.ndarray = np.zeros(3)
    q: np.ndarray = np.zeros(4)
    w: np.ndarray = np.zeros(3)

    def as_ndarray(self):
        return np.concatenate((self.p,self.v,self.q,self.w))

class MavSDKClient():
    drone: System
    state: DroneState
    state_lock : threading.Lock

    asyncio_loop : asyncio.BaseEventLoop # start an event loop to avoid conflicts with ros async callbacks
    event_loop_thread : threading.Thread

    def __init__(self):
        self.drone = System()
        self.state = DroneState()
        self.state_lock = threading.Lock()

        self.asyncio_loop = asyncio.new_event_loop()
        self.event_loop_thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.event_loop_thread.start()

    async def initialize(self):
        """
        Initialize Class
        Constructor should return fast, so async operations are moved here. 

        Current implementation returns a future while waiting for connections. So other process can occur while waiting for full connection
        """
        await self.drone.connect(system_address="udp://:14540")
        drone_connected = await self._wait_for_connection(timeout_s=15.0)
        if not drone_connected:
            print("Connection failed")
            return
        drone_armed = await self._wait_until_armable()
        if not drone_armed:
            print("Vehicle not armable")
            return
        
        print("[MAV SDK]: Arming...")
        await self.drone.action.arm()

        # Set initial setpoint before starting offboard
        # Start with hover attitude rate command
        await self.drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, 0.0))
        
        print("[MAV SDK]: Starting offboard...")
        await self.drone.offboard.start() 

        self.start_telemetry()

    def send_AttitudeRate_cmd(self, cmd : AttitudeRate):
        #TODO: verify that it is best way
        self.run_async(self.drone.offboard.set_attitude_rate(cmd))

    def send_Position_cmd(self, cmd : PositionNedYaw):
        #TODO: verify that it is best way
        self.run_async(self.drone.offboard.set_position_ned(cmd))

    def run_async(self, coroutine):
        """Schedule async coroutines safely from another thread"""
        #TODO: chat suggested this, not yet sold for this
        return asyncio.run_coroutine_threadsafe(coroutine, self.asyncio_loop)
    
    def get_latest_state(self):
        """
        get latest state.
        """
        with self.state_lock:
            return self.state.as_ndarray()
    
    def start_telemetry(self):
        """Function that start the telemetry streams with mavsdk"""

        self.run_async(self._position_listener())
        self.run_async(self._attitude_listener())
        self.run_async(self._odometry_listener())

    def _run_event_loop(self):
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_forever()

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
                self.state.q = np.array([att.w, att.x, att.y, att.z])

    async def _odometry_listener(self):
        """Expresses in Body NED frame (roll, pitch, yaw)"""
        async for odo in self.drone.telemetry.odometry():
            with self.state_lock:
                self.state.w = np.array([odo.angular_velocity_body.roll_rad_s,
                                        odo.angular_velocity_body.pitch_rad_s,
                                         odo.angular_velocity_body.yaw_rad_s])
   
    async def _wait_for_connection(self,timeout_s: float = 10.0):
        """Wait for MAVSDK connection."""
        print("[MAV SDK]: Waiting connection...")
        t0 = time.time()
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Connected to vehicle")
                return True
            if time.time() - t0 > timeout_s:
                print("Connection timeout")
                return False
            await asyncio.sleep(0.1)

    async def _wait_until_armable(self, timeout_s: float = 30.0):
        """Wait until vehicle health indicates armable state."""
        print("[MAV SDK]: Waiting for vehicle to be armable...")
        t0 = time.time()
        async for health in self.drone.telemetry.health():
            print(f"Health: global={health.is_global_position_ok}, "
                f"local={health.is_local_position_ok}, "
                f"is_armable={health.is_armable}")
            if health.is_armable:
                return True
            if time.time() - t0 > timeout_s:
                print("Timed out waiting for armable")
                return False
            await asyncio.sleep(0.5)
        return False
    
    
    
     # TODO: disarm


# Testing MAVSDK client

def main(args=None):
    mav_sdk_client = MavSDKClient()
    init_future = mav_sdk_client.run_async(mav_sdk_client.initialize())
    init_future.result() # wait for initialization


    while True:
        state =  mav_sdk_client.get_latest_state()
        print(f"State is {state}")
        

if __name__ == '__main__':
    main()