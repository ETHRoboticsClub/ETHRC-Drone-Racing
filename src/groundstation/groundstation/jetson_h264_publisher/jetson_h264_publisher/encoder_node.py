#!/usr/bin/env python3
"""
Jetson Orin NX hardware H.264 encoder ROS 2 publisher.

Captures from a CSI camera via nvarguscamerasrc, encodes with the Jetson's
hardware encoder (nvv4l2h264enc), and publishes h264_msgs/Packet on a topic
compatible with h264_image_transport subscribers.

Tested on JetPack 6 / Ubuntu 22.04 / ROS 2 Humble.
"""

import threading
from typing import Optional

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # noqa: E402

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from h264_msgs.msg import Packet


class JetsonH264Publisher(Node):
    def __init__(self):
        super().__init__("jetson_h264_publisher")

        # ---- Parameters ----
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("bitrate", 4_000_000)        # bits per second
        self.declare_parameter("peak_bitrate", 6_000_000)
        self.declare_parameter("iframe_interval", 15)        # frames; lower = faster sync, more bandwidth
        self.declare_parameter("control_rate", 1)            # 0=disable, 1=VBR, 2=CBR
        self.declare_parameter("preset_level", 1)            # 1=UltraFast .. 4=Slow
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("topic", "image_raw/h264")
        self.declare_parameter("flip_method", 0)             # nvvidconv flip-method
        # If your sensor needs a non-default exposure/gain, expose those here too.

        p = self.get_parameter
        self.frame_id = p("frame_id").value
        topic = p("topic").value

        # Best-effort QoS makes sense for video over a radio link;
        # use sensor-data-style settings.
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Packet, topic, qos)
        self._seq = 0

        # ---- GStreamer pipeline ----
        Gst.init(None)
        pipeline_str = self._build_pipeline()
        self.get_logger().info(f"GStreamer pipeline:\n  {pipeline_str}")
        self.pipeline = Gst.parse_launch(pipeline_str)

        appsink = self.pipeline.get_by_name("sink")
        appsink.set_property("emit-signals", True)
        appsink.set_property("sync", False)
        appsink.set_property("max-buffers", 2)
        appsink.set_property("drop", True)
        appsink.connect("new-sample", self._on_new_sample)

        # Bus thread to surface errors / EOS
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message", self._on_bus_message)

        self._loop = GLib.MainLoop()
        self._loop_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._loop_thread.start()

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start GStreamer pipeline")
        self.get_logger().info("Encoder pipeline running.")

    def _build_pipeline(self) -> str:
        p = self.get_parameter
        return (
            f"nvarguscamerasrc sensor-id={p('sensor_id').value} ! "
            f"video/x-raw(memory:NVMM), width={p('width').value}, "
            f"height={p('height').value}, framerate={p('framerate').value}/1, "
            f"format=NV12 ! "
            f"nvvidconv flip-method={p('flip_method').value} ! "
            f"video/x-raw(memory:NVMM), format=NV12 ! "
            f"nvv4l2h264enc "
            f"bitrate={p('bitrate').value} "
            f"peak-bitrate={p('peak_bitrate').value} "
            f"control-rate={p('control_rate').value} "
            f"preset-level={p('preset_level').value} "
            f"iframeinterval={p('iframe_interval').value} "
            f"insert-sps-pps=true "
            f"maxperf-enable=true "
            f"profile=4 ! "                     # 4 = High
            f"h264parse config-interval=1 ! "
            f"video/x-h264, stream-format=byte-stream, alignment=au ! "
            f"appsink name=sink"
        )

    def _on_new_sample(self, sink) -> int:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        try:
            msg = Packet()
            now = self.get_clock().now().to_msg()
            # Prefer PTS as capture timestamp; fall back to wall clock.
            pts_ns = buf.pts
            if pts_ns != Gst.CLOCK_TIME_NONE:
                # Pipeline-relative PTS — fine for relative timing within a session.
                msg.header.stamp.sec = int(pts_ns // 1_000_000_000)
                msg.header.stamp.nanosec = int(pts_ns % 1_000_000_000)
            else:
                msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.seq = self._seq
            self._seq = (self._seq + 1) & 0xFFFFFFFF
            msg.data = bytes(mapinfo.data)  # h264_msgs/Packet.data is uint8[]
            self.pub.publish(msg)
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            self.get_logger().error(f"GStreamer error: {err.message} ({dbg})")
            rclpy.shutdown()
        elif t == Gst.MessageType.EOS:
            self.get_logger().warning("GStreamer EOS")
            rclpy.shutdown()
        elif t == Gst.MessageType.WARNING:
            err, dbg = message.parse_warning()
            self.get_logger().warning(f"GStreamer warning: {err.message} ({dbg})")

    def destroy_node(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
        except Exception:
            pass
        try:
            self._loop.quit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[JetsonH264Publisher] = None
    try:
        node = JetsonH264Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
