import asyncio
import json
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CameraInfo, Image, Imu
from std_msgs.msg import Float64, Float64MultiArray
import uvicorn


CAMERA_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
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

        self.create_subscription(Imu, '/imu0', self._imu_cb, 10)
        self.create_subscription(BatteryState, '/battery', self._battery_cb, 10)
        self.create_subscription(CameraInfo, '/cam0/camera_info', self._camera_info_cb, CAMERA_QOS)
        self.create_subscription(Image, '/cam0/image_raw', self._image_cb, CAMERA_QOS)
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
        self.state.update('image', {
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        })
        if msg.encoding == 'mono8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            bgr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        elif msg.encoding == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        else:
            return
        _, jpeg = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        self.state.update_frame(jpeg.tobytes())

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


def build_app(state: TelemetryState, static_dir: Path) -> FastAPI:
    app = FastAPI(title='ETHRC Drone Groundstation')

    app.mount('/static', StaticFiles(directory=static_dir), name='static')

    @app.get('/')
    async def index():
        return FileResponse(static_dir / 'index.html')

    @app.get('/api/telemetry')
    async def telemetry():
        return state.snapshot()

    @app.websocket('/ws')
    async def ws_stream(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_text(json.dumps(state.snapshot()))
                await asyncio.sleep(1.0 / 30.0)
        except WebSocketDisconnect:
            pass

    @app.get('/video')
    async def video_stream():
        async def mjpeg():
            try:
                while True:
                    frame = state.latest_frame()
                    if frame is not None:
                        yield (
                            b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n'
                            + frame
                            + b'\r\n'
                        )
                    await asyncio.sleep(1.0 / 30.0)
            except asyncio.CancelledError:
                pass
        return StreamingResponse(
            mjpeg(),
            media_type='multipart/x-mixed-replace; boundary=frame',
        )

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
        uvicorn.run(build_app(state, static_dir), host=host, port=port, log_level='warning')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
