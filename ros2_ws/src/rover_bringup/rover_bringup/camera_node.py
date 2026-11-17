"""camera_node: webcam -> lane paint + obstacles on the floor (rover frame).

Publishes (std_msgs, so nothing custom to build):
  camera/obstacles  Float32MultiArray [x0, y0, x1, y1, ...]  floor points of coloured or
                    dark things (cars), metres, rover frame (x forward, y left, centre origin)
  camera/lines      Float32MultiArray, same layout: white paint points (thinned)
  camera/info       String JSON: fps, counts
Writes ~/rover_data/camera/live.png (with the detections drawn on) for the dashboard.
"""
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String

from rover_bringup import camera_vision as cv
from rover_bringup.v4l2cam import Camera, write_png

LIVE = os.path.expanduser('~/rover_data/camera/live.png')


def _flat(pts, cap):
    if len(pts) > cap:
        pts = pts[np.linspace(0, len(pts) - 1, cap).astype(int)]
    return [float(v) for v in np.round(pts, 3).ravel()]


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('device', '')
        self.declare_parameter('rate', 10.0)          # processed frames per second
        self.declare_parameter('live_period', 0.5)    # s between dashboard pictures
        self.obs_pub = self.create_publisher(Float32MultiArray, 'camera/obstacles', qos_profile_sensor_data)
        self.line_pub = self.create_publisher(Float32MultiArray, 'camera/lines', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(String, 'camera/info', 10)
        calib = cv.load_calib()
        self.model = cv.CameraModel(calib)
        self.det = cv.Detector(self.model)
        os.makedirs(os.path.dirname(LIVE), exist_ok=True)
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        dev = self.get_parameter('device').value or None
        period = 1.0 / max(1.0, float(self.get_parameter('rate').value))
        live_period = float(self.get_parameter('live_period').value)
        cam = None
        last_live, n, t_count, fps = 0.0, 0, time.monotonic(), 0.0
        while self.running and rclpy.ok():
            if cam is None:
                try:
                    cam = Camera(dev, self.model.c['width'], self.model.c['height'])
                    self.get_logger().info(f'camera {cam.dev} {cam.width}x{cam.height} streaming')
                except OSError as e:
                    self.get_logger().warn(f'camera: {e}; retrying in 3 s', throttle_duration_sec=30)
                    time.sleep(3)
                    continue
            t0 = time.monotonic()
            try:
                data = cam.read(1.0)
            except OSError as e:
                self.get_logger().warn(f'camera read failed ({e}); reopening')
                cam.close()
                cam = None
                continue
            if data is None or len(data) < cam.width * cam.height * 2:
                continue
            Y, U, V = cv.yuyv_planes(data, cam.width, cam.height)
            res = self.det.run(Y, U, V)
            self.obs_pub.publish(Float32MultiArray(data=_flat(res['obstacles'], 320)))
            self.line_pub.publish(Float32MultiArray(data=_flat(res['lines'], 400)))
            n += 1
            now = time.monotonic()
            if now - t_count >= 2.0:
                fps, n, t_count = n / (now - t_count), 0, now
                self.info_pub.publish(String(data=json.dumps({
                    'fps': round(fps, 1), 'obstacles': int(len(res['obstacles'])),
                    'lines': int(len(res['lines'])), 'dev': cam.dev})))
            if now - last_live >= live_period:
                last_live = now
                try:
                    write_png(LIVE, cv.overlay(Y, U, V, res))
                except OSError:
                    pass
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        if cam is not None:
            cam.close()

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
