import asyncio
from threading import Lock, Thread
from dataclasses import dataclass
from typing import Optional

import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, AttitudeRate


@dataclass
class DroneState:
    position: np.ndarray = np.zeros(3)
    velocity: np.ndarray = np.zeros(3)
    quaternion: np.ndarray = np.array([1, 0, 0, 0])
    angular_velocity: np.ndarray = np.zeros(3)

    def to_vector(self):
        return np.concatenate((
            self.position,
            self.velocity,
            self.quaternion,
            self.angular_velocity
        ))


class MavsdkClient:
    def __init__(self, connection_string: str = 'udpin://0.0.0.0:14540'):
        self.connection_string = connection_string
        self.drone = System()

        self.current_state = DroneState()
        self.state_lock = Lock()

        self.is_connected = False
        self.is_armed = False
        self.is_offboard = False
        self.health_ok = False

        self.loop = asyncio.new_event_loop()
        self.async_thread = Thread(target=self._run_async_loop, daemon=True)
        self.async_thread.start()

        # schedule initialization
        asyncio.run_coroutine_threadsafe(self._initialize_mavsdk(), self.loop)

    def _run_async_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _initialize_mavsdk(self):
        try:
            await self.drone.connect(system_address=self.connection_string)

            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    self.is_connected = True
                    break

            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    self.health_ok = True
                    break

            await self._start_telemetry()
            asyncio.create_task(self._monitor_connection())
            asyncio.create_task(self._monitor_armed_state())
            asyncio.create_task(self._monitor_health())

        except Exception as e:
            print(f"MAVSDK init failed: {e}")

    async def _start_telemetry(self):
        asyncio.create_task(self._subscribe_position_velocity())
        asyncio.create_task(self._subscribe_attitude())
        asyncio.create_task(self._subscribe_angular_velocity())

    async def _subscribe_position_velocity(self):
        async for pv in self.drone.telemetry.position_velocity_ned():
            with self.state_lock:
                self.current_state.position = np.array([
                    pv.position.north_m,
                    pv.position.east_m,
                    pv.position.down_m
                ])
                self.current_state.velocity = np.array([
                    pv.velocity.north_m_s,
                    pv.velocity.east_m_s,
                    pv.velocity.down_m_s
                ])

    async def _subscribe_attitude(self):
        async for att in self.drone.telemetry.attitude_quaternion():
            with self.state_lock:
                self.current_state.quaternion = np.array([
                    att.w, att.x, att.y, att.z
                ])

    async def _subscribe_angular_velocity(self):
        async for av in self.drone.telemetry.attitude_angular_velocity_body():
            with self.state_lock:
                self.current_state.angular_velocity = np.array([
                    av.roll_rad_s, av.pitch_rad_s, av.yaw_rad_s
                ])

    async def _monitor_connection(self):
        async for state in self.drone.core.connection_state():
            self.is_connected = state.is_connected

    async def _monitor_armed_state(self):
        async for is_armed in self.drone.telemetry.armed():
            self.is_armed = is_armed

    async def _monitor_health(self):
        async for health in self.drone.telemetry.health():
            self.health_ok = health.is_global_position_ok and health.is_home_position_ok

    async def arm_and_start_offboard(self):
        try:
            await self.drone.action.arm()
            # send initial zero-rate setpoints
            init = AttitudeRate(0.0, 0.0, 0.0, 0.0)
            for _ in range(10):
                await self.drone.offboard.set_attitude_rate(init)
                await asyncio.sleep(0.1)
            await self.drone.offboard.start()
            self.is_offboard = True
        except OffboardError as e:
            print(f"Offboard failed: {e}")
            try:
                await self.drone.action.disarm()
            except Exception:
                pass
        except Exception as e:
            print(f"Arm/offboard failed: {e}")

    async def send_attitude_rate(self, thrust: float, body_rates: np.ndarray):
        try:
            att = AttitudeRate(
                roll_deg_s=float(np.degrees(body_rates[0])),
                pitch_deg_s=float(np.degrees(body_rates[1])),
                yaw_deg_s=float(np.degrees(body_rates[2])),
                thrust_value=float(thrust)
            )
            await self.drone.offboard.set_attitude_rate(att)
        except Exception as e:
            print(f"Failed to send attitude rate: {e}")

    def run_coroutine(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)
