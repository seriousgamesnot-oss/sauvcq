#!/usr/bin/env python3
import collections, time
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from sauvc_interfaces.msg import GateDetection

WIN = 10.0
WHITE = (255, 255, 255)


class Panel(Node):
    def __init__(self):
        super().__init__('gate_panel')
        self.declare_parameter('debug_topic', '/gate/debug')
        self.br = CvBridge()
        self.frame = np.zeros((540, 960, 3), np.uint8)
        self.det, self.lock, self.pos = (collections.deque() for _ in range(3))
        self.last_lock, self.drops, self.last_gate_t = False, 0, 0.0
        self.scores = collections.deque()
        dbg = self.get_parameter('debug_topic').value
        self.create_subscription(Image, dbg, self.on_debug, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/gate/detections', self.on_det, 10)
        self.create_subscription(Bool, '/gate/lock', self.on_lock, 10)
        self.create_subscription(GateDetection, '/varuna/gate_detection', self.on_gate, 10)
        self.pub = self.create_publisher(Image, '/gate/panel', 1)
        self.create_timer(0.1, self.draw)

    def now(self):
        return time.monotonic()

    def on_debug(self, m):
        try:
            self.frame = cv2.resize(self.br.imgmsg_to_cv2(m, 'bgr8'), (960, 540))
        except Exception as e:
            self.get_logger().warn(f'debug image: {e}', throttle_duration_sec=5.0)

    def on_det(self, m):
        self.det.append((self.now(), len(m.detections) > 0))
        try:
            best = max((float(r.hypothesis.score) for d in m.detections for r in d.results), default=None)
            if best is not None:
                self.scores.append((self.now(), best))
        except Exception:
            pass

    def on_lock(self, m):
        if self.last_lock and not m.data:
            self.drops += 1
        self.last_lock = bool(m.data)
        self.lock.append((self.now(), bool(m.data)))

    def on_gate(self, m):
        if m.has_position:
            t = self.now()
            self.last_gate_t = t
            self.pos.append((t, float(m.confidence), m.position.x, m.position.y))

    def draw(self):
        t = self.now()
        for dq in (self.det, self.lock, self.pos, self.scores):
            while dq and t - dq[0][0] > WIN:
                dq.popleft()
        det_hz = len(self.det) / WIN
        box_pct = 100.0 * sum(1 for _, b in self.det if b) / max(len(self.det), 1)
        lock_pct = 100.0 * sum(1 for _, b in self.lock if b) / max(len(self.lock), 1)
        conf = float(np.mean([p[1] for p in self.pos])) if self.pos else 0.0
        jx = float(np.std([p[2] for p in self.pos])) if len(self.pos) > 3 else 0.0
        jy = float(np.std([p[3] for p in self.pos])) if len(self.pos) > 3 else 0.0
        age = t - self.last_gate_t if self.last_gate_t else None
        ds = float(np.mean([x for _, x in self.scores])) if self.scores else 0.0
        if not self.last_lock:
            verdict, col = 'NO GATE', (0, 0, 255)
        elif lock_pct < 80 or jx > 0.5:
            verdict, col = 'FLICKERY', (0, 200, 255)
        else:
            verdict, col = 'LOCKED', (0, 200, 0)
        side = np.full((540, 330, 3), 30, np.uint8)
        rows = [(verdict, col, 1.0),
                (f'detector {det_hz:5.1f} Hz', WHITE, .6),
                (f'box rate {box_pct:5.0f} %', WHITE, .6),
                (f'lock     {lock_pct:5.0f} %  (10 s)', WHITE, .6),
                (f'drops    {self.drops}', WHITE, .6),
                (f'det score {ds:4.2f} (top box)', WHITE, .6),
                (f'bridge conf {conf:4.2f} (fixed)', WHITE, .6),
                (f'jitter x {jx:.2f} m  y {jy:.2f} m', WHITE, .6),
                (f'last pos {age:4.1f} s ago' if age is not None else 'last pos  never', WHITE, .6)]
        y = 40
        for txt, c, sc in rows:
            cv2.putText(side, txt, (12, y), cv2.FONT_HERSHEY_SIMPLEX, sc, c,
                        2 if sc == 1.0 else 1, cv2.LINE_AA)
            y += 42 if sc == 1.0 else 32
        strip = np.zeros(100, bool)
        for ts, ok in self.lock:
            i = int((t - ts) / WIN * 100)
            if ok and 0 <= i < 100:
                strip[99 - i] = True
        for i, ok in enumerate(strip):
            cv2.rectangle(side, (12 + i * 3, 480), (14 + i * 3, 520),
                          (0, 200, 0) if ok else (60, 60, 60), -1)
        cv2.putText(side, 'lock history (10 s)', (12, 470),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (200, 200, 200), 1)
        self.pub.publish(self.br.cv2_to_imgmsg(np.hstack([self.frame, side]), 'bgr8'))


def main():
    rclpy.init()
    rclpy.spin(Panel())


if __name__ == '__main__':
    main()
