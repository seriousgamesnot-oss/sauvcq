#!/usr/bin/env python3
"""Pixel-level gate perception -> sauvc_interfaces/GateDetection.

Synthesises a body-frame position (x fwd, y left) from the horizontal steering
error + rough range, so the BT's IsGateVisible/MoveToRelativePose work unchanged.
Publishes ONLY while locked with fresh steering and range, so the BT's
max_age_s check times out naturally when the gate is lost.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float32
from sauvc_interfaces.msg import GateDetection


class GateBridgePixel(Node):
    def __init__(self):
        super().__init__('gate_bridge_pixel')
        P = self.declare_parameter
        P('steering_topic', '/gate/steering_filtered')
        P('range_topic', '/gate/range_rough')
        P('lock_topic', '/gate/lock')
        P('output_topic', '/varuna/gate_detection')
        P('camera_id', 'front')
        P('frame_id', 'base_link')
        P('hfov_deg', 80.0)        # front camera horizontal FOV
        P('image_width', 0.0)      # 0 => steering.x already normalised to [-1,1]
        P('x_sign', 1.0)           # flip if gate-right gives negative x
        P('min_range_m', 0.3)
        P('max_range_m', 10.0)
        P('lock_confidence', 0.9)
        P('max_input_age_s', 0.5)

        g = lambda n: self.get_parameter(n).value
        self.cam, self.frame = g('camera_id'), g('frame_id')
        self.tan_half = math.tan(math.radians(g('hfov_deg')) / 2.0)
        self.img_w, self.sign = g('image_width'), g('x_sign')
        self.rmin, self.rmax = g('min_range_m'), g('max_range_m')
        self.conf, self.max_age = g('lock_confidence'), g('max_input_age_s')

        self.locked, self.t_lock = False, None
        self.range_m, self.t_range = None, None

        self.pub = self.create_publisher(GateDetection, g('output_topic'), 10)
        self.create_subscription(Bool, g('lock_topic'), self.cb_lock, 10)
        self.create_subscription(Float32, g('range_topic'), self.cb_range, 10)
        self.create_subscription(Vector3Stamped, g('steering_topic'), self.cb_steer, 10)

    def _age(self, t):
        return float('inf') if t is None else (self.get_clock().now() - t).nanoseconds * 1e-9

    def cb_lock(self, m):
        self.locked, self.t_lock = bool(m.data), self.get_clock().now()

    def cb_range(self, m):
        self.range_m, self.t_range = float(m.data), self.get_clock().now()

    def cb_steer(self, m):
        if not (self.locked and self._age(self.t_lock) < 1.0):
            return
        if self._age(self.t_range) > self.max_age or self.range_m is None:
            self.get_logger().warn('no fresh range_rough; not publishing', throttle_duration_sec=2.0)
            return
        r = min(max(self.range_m, self.rmin), self.rmax)

        x = self.sign * m.vector.x
        if self.img_w > 0.0:                      # pixels from image centre -> [-1,1]
            x = x / (self.img_w / 2.0)
        x = max(-1.0, min(1.0, x))
        bearing = math.atan(x * self.tan_half)    # +ve = gate to the right

        d = GateDetection()
        d.header.stamp = m.header.stamp
        d.header.frame_id = self.frame
        d.camera_id = self.cam
        d.confidence = float(self.conf)
        d.has_position = True
        d.position.x = r * math.cos(bearing)
        d.position.y = -r * math.sin(bearing)     # body y is LEFT
        d.position.z = 0.0                        # depth is held by MaintainDepth
        d.orientation.w = 1.0                     # no gate orientation from pixel level
        d.has_bbox = False
        self.pub.publish(d)


def main():
    rclpy.init()
    n = GateBridgePixel()
    try:
        rclpy.spin(n)
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
