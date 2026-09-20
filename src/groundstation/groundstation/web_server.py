import asyncio
import json
import math
import threading
from pathlib import Path

import av as pyav
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import Bool, Float64, Float64MultiArray
import uvicorn


CAMERA_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)

# image_rgb and image_h264 are published with RELIABLE QoS by the camera node
IMAGE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
)

# Latch the killswitch trigger so late subscribers (e.g., the bridge restarting) see it.
KILLSWITCH_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class TelemetryState:
    """Thread-safe latest-value store shared between ROS and FastAPI threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data: dict = {}
        self._frame: bytes | None = None

    def update(self, key: str, value):
        with self._lock:
            self._data[key] = value

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._data)

    def update_frame(self, jpeg: bytes):
        with self._lock:
            self._frame = jpeg

    def latest_frame(self) -> bytes | None:
        with self._lock:
            return self._frame


class GroundstationNode(Node):
    def __init__(self, state: TelemetryState):
        super().__init__('groundstation_web_server')
        self.state = state
        self._h264_codec = pyav.CodecContext.create('h264', 'r')
        self._h264_active = False  # set True once first H264 frame decodes; suppresses raw RGB
        self._killswitch_sub = None  # toggled by /api/killswitch/bounds
        self._killswitch_live = None  # running {'min': {x,y,z}, 'max': {x,y,z}} while recording
        self.killswitch_bounds = None  # last finalized bounds after recording stops
        self._killswitch_monitor_sub = None  # toggled by /api/killswitch/activate
        self._killswitch_triggered = False  # latched once a violation is detected

        # Publisher: send True to trigger Betaflight failsafe; subscriber lives in msp_bridge.
        self.killswitch_pub = self.create_publisher(Bool, '/drone/killswitch', KILLSWITCH_QOS)

        self.create_subscription(Imu, '/imu0', self._imu_cb, 10)
        self.create_subscription(BatteryState, '/battery', self._battery_cb, 10)
        self.create_subscription(CameraInfo, '/cam0/camera_info', self._camera_info_cb, CAMERA_QOS)
        self.create_subscription(Image, '/cam0/image_rgb', self._image_cb, IMAGE_QOS)
        self.create_subscription(CompressedImage, '/cam0/image_h264', self._image_h264_cb, IMAGE_QOS)
        self.create_subscription(Float64, '/drone/throttle', self._throttle_cb, 10)
        self.create_subscription(Float64MultiArray, '/drone_attitude_cmd', self._attitude_cmd_cb, 10)
        self.create_subscription(PoseStamped, '/drone_current_target', self._current_target_cb, 10)
        self.create_subscription(Float64MultiArray, '/drone_position_cmd', self._position_cmd_cb, 10)
        self.create_subscription(Float64MultiArray, '/motor_rpm', self._motor_rpm_cb, 10)

        self.get_logger().info('GroundstationNode started — subscribed to all topics')

    def _imu_cb(self, msg: Imu):
        o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        self.state.update('imu', {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w},
            'angular_velocity': {'x': w.x, 'y': w.y, 'z': w.z},
            'linear_acceleration': {'x': a.x, 'y': a.y, 'z': a.z},
        })

    def _battery_cb(self, msg: BatteryState):
        self.state.update('battery', {
            'voltage': msg.voltage,
            'percentage': msg.percentage,
        })

    def _camera_info_cb(self, msg: CameraInfo):
        self.state.update('camera_info', {
            'width': msg.width,
            'height': msg.height,
            'distortion_model': msg.distortion_model,
        })

    def _image_cb(self, msg: Image):
        if self._h264_active:
            return
        self.state.update('image', {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        })
        try:
            if msg.encoding == 'mono8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                bgr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            elif msg.encoding == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            else:
                self.get_logger().warning(
                    f"Unsupported image encoding '{msg.encoding}'",
                    throttle_duration_sec=10.0,
                )
                return
            _, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.state.update_frame(jpeg.tobytes())
        except Exception as exc:
            self.get_logger().warning(f'image_rgb decode error: {exc}', throttle_duration_sec=5.0)

    def _image_h264_cb(self, msg: CompressedImage):
        self.state.update('image', {
            'width': 0,
            'height': 0,
            'encoding': 'h264',
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        })
        try:
            data = bytes(msg.data)
            packets = self._h264_codec.parse(data)
            for packet in packets:
                frames = self._h264_codec.decode(packet)
                for frame in frames:
                    bgr = frame.to_ndarray(format='bgr24')
                    self.state.update('image', {
                        'width': frame.width,
                        'height': frame.height,
                        'encoding': 'h264',
                        'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    })
                    self._h264_active = True
                    _, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.state.update_frame(jpeg.tobytes())
                    return
        except Exception as exc:
            self.get_logger().warning(f'H264 decode error: {exc}', throttle_duration_sec=5.0)

    def _throttle_cb(self, msg: Float64):
        self.state.update('throttle', msg.data)

    def _attitude_cmd_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        self.state.update('attitude_cmd', {
            'roll': d[0] if len(d) > 0 else None,
            'pitch': d[1] if len(d) > 1 else None,
            'yaw': d[2] if len(d) > 2 else None,
            'thrust': d[3] if len(d) > 3 else None,
        })

    def _current_target_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.state.update('current_target', {'x': p.x, 'y': p.y, 'z': p.z})

    def _position_cmd_cb(self, msg: Float64MultiArray):
        self.state.update('position_cmd', list(msg.data))

    def _motor_rpm_cb(self, msg: Float64MultiArray):
        self.state.update('motor_rpm', list(msg.data))

    def configure_killswitch_bounds(self) -> dict:
        """Toggle subscription to /vicon/zephyrus/zephyrus (PoseStamped).

        First click: start subscribing and tracking per-axis min/max.
        Second click: stop subscribing and store the recorded bounds.
        """
        if self._killswitch_sub is None:
            self._killswitch_live = {
                'min': {'x': math.inf,  'y': math.inf,  'z': math.inf},
                'max': {'x': -math.inf, 'y': -math.inf, 'z': -math.inf},
                'samples': 0,
            }
            self._killswitch_sub = self.create_subscription(
                PoseStamped,
                '/vicon/zephyrus/zephyrus',
                self._killswitch_pose_cb,
                10,
            )
            self.state.update('killswitch_bounds', {'active': True, 'bounds': None})
            self.get_logger().info('Killswitch bounds: recording started')
            return {'active': True}

        self.destroy_subscription(self._killswitch_sub)
        self._killswitch_sub = None

        live = self._killswitch_live
        self._killswitch_live = None
        if live is None or live['samples'] == 0:
            self.get_logger().warning('Killswitch bounds: no samples recorded')
            self.state.update('killswitch_bounds', {'active': False, 'bounds': None})
            return {'active': False, 'bounds': None, 'samples': 0}

        self.killswitch_bounds = {'min': live['min'], 'max': live['max']}
        self.state.update('killswitch_bounds', {
            'active': False,
            'bounds': self.killswitch_bounds,
            'samples': live['samples'],
        })
        self.get_logger().info(
            f"Killswitch bounds stored ({live['samples']} samples): "
            f"min={self.killswitch_bounds['min']} max={self.killswitch_bounds['max']}"
        )
        return {'active': False, 'bounds': self.killswitch_bounds, 'samples': live['samples']}

    def _killswitch_pose_cb(self, msg: PoseStamped):
        """Update per-axis min/max while recording is active."""
        live = self._killswitch_live
        if live is None:
            return
        p = msg.pose.position
        for axis, val in (('x', p.x), ('y', p.y), ('z', p.z)):
            if val < live['min'][axis]:
                live['min'][axis] = val
            if val > live['max'][axis]:
                live['max'][axis] = val
        live['samples'] += 1

    def activate_killswitch(self) -> dict:
        """Toggle monitoring of /vicon/zephyrus/zephyrus against recorded bounds.

        While active, every incoming pose is compared to self.killswitch_bounds;
        any axis outside [min, max] triggers a printed warning.
        """
        if self._killswitch_monitor_sub is None:
            if self.killswitch_bounds is None:
                self.get_logger().warning('Killswitch activate refused: no bounds recorded yet')
                return {'active': False, 'error': 'No bounds recorded. Configure killswitch bounds first.'}
            self._killswitch_triggered = False
            self.killswitch_pub.publish(Bool(data=False))
            self._killswitch_monitor_sub = self.create_subscription(
                PoseStamped,
                '/vicon/zephyrus/zephyrus',
                self._killswitch_monitor_cb,
                10,
            )
            self.state.update('killswitch_active', True)
            self.get_logger().info(
                f'Killswitch activated. Bounds: min={self.killswitch_bounds["min"]} '
                f'max={self.killswitch_bounds["max"]}'
            )
            return {'active': True, 'bounds': self.killswitch_bounds}

        self.destroy_subscription(self._killswitch_monitor_sub)
        self._killswitch_monitor_sub = None
        self._killswitch_triggered = False
        self.killswitch_pub.publish(Bool(data=False))
        self.state.update('killswitch_active', False)
        self.get_logger().info('Killswitch deactivated')
        return {'active': False}

    def _killswitch_monitor_cb(self, msg: PoseStamped):
        """Print a warning whenever any axis leaves the recorded bounds."""
        bounds = self.killswitch_bounds
        if bounds is None:
            return
        p = msg.pose.position
        violations = []
        for axis, val in (('x', p.x), ('y', p.y), ('z', p.z)):
            lo, hi = bounds['min'][axis], bounds['max'][axis]
            if val < lo or val > hi:
                violations.append(f'{axis}={val:.3f} not in [{lo:.3f}, {hi:.3f}]')
        if violations:
            print('[KILLSWITCH] OUT OF BOUNDS: ' + '; '.join(violations), flush=True)
            if not self._killswitch_triggered:
                self._killswitch_triggered = True
                self.killswitch_pub.publish(Bool(data=True))
                self.state.update('killswitch_triggered', True)
                self.get_logger().error(
                    '[KILLSWITCH] Triggered Betaflight failsafe via /drone/killswitch'
                )


def build_app(state: TelemetryState, node: 'GroundstationNode', static_dir: Path) -> FastAPI:
    app = FastAPI(title='ETHRC Drone Groundstation')

    app.mount('/static', StaticFiles(directory=static_dir), name='static')

    @app.get('/')
    async def index():
        return FileResponse(static_dir / 'index.html')

    @app.get('/api/telemetry')
    async def telemetry():
        return state.snapshot()

    @app.post('/api/killswitch/bounds')
    async def configure_killswitch_bounds():
        return node.configure_killswitch_bounds()

    @app.post('/api/killswitch/activate')
    async def activate_killswitch():
        return node.activate_killswitch()

    @app.websocket('/ws')
    async def ws_stream(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_text(json.dumps(state.snapshot()))
                await asyncio.sleep(1.0 / 30.0)
        except WebSocketDisconnect:
            pass

    @app.get('/snapshot')
    async def snapshot():
        frame = state.latest_frame()
        if frame is None:
            return Response(status_code=204)
        return Response(content=frame, media_type='image/jpeg')

    return app


def main():
    rclpy.init()

    state = TelemetryState()
    node = GroundstationNode(state)

    node.declare_parameter('host', '127.0.0.1')
    node.declare_parameter('port', 8080)
    host = node.get_parameter('host').get_parameter_value().string_value
    port = node.get_parameter('port').get_parameter_value().integer_value

    static_dir = Path(get_package_share_directory('groundstation')) / 'static'

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    node.get_logger().info(f'Web UI at http://{host}:{port}')

    try:
        uvicorn.run(build_app(state, node, static_dir), host=host, port=port, log_level='warning')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
