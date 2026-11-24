#!/usr/bin/env python3
"""Graba a CSV las poses de odometría, EKF y ground truth (todas en el frame map),
la diagonal de la covarianza del EKF y la cantidad de postes detectados.
Uso: sampler.py <salida.csv> <duracion_seg_sim>"""
import sys, math, csv
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseWithCovarianceStamped
from robmovil_msgs.msg import LandmarkArray

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

class Sampler(Node):
    def __init__(self, out, dur):
        super().__init__('ekf_sampler', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buf = Buffer(); self.tl = TransformListener(self.buf, self)
        self.P = (float('nan'),) * 3; self.nl = 0
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.on_pose, 10)
        self.create_subscription(LandmarkArray, '/landmarks', self.on_lm, 10)
        self.f = open(out, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['t', 'odo_x', 'odo_y', 'odo_th', 'ekf_x', 'ekf_y', 'ekf_th',
                         'gt_x', 'gt_y', 'gt_th', 'P_xx', 'P_yy', 'P_thth', 'n_postes'])
        self.dur = dur; self.t0 = None
        self.create_timer(0.1, self.tick)

    def on_pose(self, m):
        c = m.pose.covariance; self.P = (c[0], c[7], c[35])

    def on_lm(self, m):
        self.nl = len(m.landmarks)

    def pose(self, child, when=None):
        tf = self.buf.lookup_transform('map', child, when if when is not None else rclpy.time.Time())
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw(t.rotation)), rclpy.time.Time.from_msg(tf.header.stamp)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None: self.t0 = now
        try:
            # las tres poses en el MISMO instante: el más reciente que tengan en común
            stamps = [self.pose(c)[1] for c in ('base_link', 'base_link_ekf', 'base_link_gt')]
            when = min(stamps, key=lambda s: s.nanoseconds)
            o, e, g = (self.pose(c, when)[0] for c in ('base_link', 'base_link_ekf', 'base_link_gt'))
        except Exception:
            return
        self.w.writerow([f'{when.nanoseconds / 1e9 - self.t0:.2f}', *[f'{v:.4f}' for v in (*o, *e, *g)],
                         *[f'{v:.6f}' for v in self.P], self.nl])
        if now - self.t0 >= self.dur:
            self.f.close(); raise SystemExit

rclpy.init()
n = Sampler(sys.argv[1], float(sys.argv[2]))
try:
    rclpy.spin(n)
except SystemExit:
    pass
