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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

class Sampler(Node):
    def __init__(self, out, dur):
        super().__init__('ekf_sampler', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buf = Buffer(); self.tl = TransformListener(self.buf, self)
        self.P = (float('nan'),) * 3; self.nl = 0
        self.goal = None; self.cmd = (0.0, 0.0, 0.0); self.out = out; self.path_msg = None
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.on_pose, 10)
        self.create_subscription(LandmarkArray, '/landmarks', self.on_lm, 10)
        # setpoint del lazo cerrado y comandos (punto 3.5); en otras corridas quedan vacios
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Twist, '/robot/cmd_vel', self.on_cmd, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/ground_truth/target_path', self.on_path, latched)
        self.f = open(out, 'w', newline=''); self.w = csv.writer(self.f)
        self.w.writerow(['t', 'odo_x', 'odo_y', 'odo_th', 'ekf_x', 'ekf_y', 'ekf_th',
                         'gt_x', 'gt_y', 'gt_th', 'P_xx', 'P_yy', 'P_thth', 'n_postes',
                         'goal_x', 'goal_y', 'goal_th', 'cmd_vx', 'cmd_vy', 'cmd_w'])
        self.dur = dur; self.t0 = None
        self.create_timer(0.1, self.tick)

    def on_pose(self, m):
        c = m.pose.covariance; self.P = (c[0], c[7], c[35])

    def on_lm(self, m):
        self.nl = len(m.landmarks)

    def on_goal(self, m):
        self.goal = (m.header.frame_id, m.pose.position.x, m.pose.position.y, yaw(m.pose.orientation))

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def to_map(self, frame, x, y, th, when=None):
        """Pasa una pose 2D del frame dado al frame map usando TF"""
        if frame in ('', 'map'):
            return x, y, th
        t = self.buf.lookup_transform('map', frame, when if when is not None else rclpy.time.Time()).transform
        a = yaw(t.rotation)
        return (t.translation.x + math.cos(a) * x - math.sin(a) * y,
                t.translation.y + math.sin(a) * x + math.cos(a) * y, a + th)

    def on_path(self, m):
        self.path_msg = m

    def save_path(self):
        """Guarda el cuadrado objetivo (en map) una sola vez, cuando TF ya puede transformarlo"""
        m = self.path_msg
        try:
            pts = [self.to_map(m.header.frame_id, p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)) for p in m.poses]
        except Exception:
            return
        with open(self.out.replace('.csv', '_path.csv'), 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['x', 'y', 'th']); w.writerows([[f'{v:.4f}' for v in p] for p in pts])
        self.path_msg = None

    def pose(self, child, when=None):
        tf = self.buf.lookup_transform('map', child, when if when is not None else rclpy.time.Time())
        t = tf.transform
        return (t.translation.x, t.translation.y, yaw(t.rotation)), rclpy.time.Time.from_msg(tf.header.stamp)

    def tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t0 is None: self.t0 = now
        if self.path_msg is not None: self.save_path()
        try:
            # las tres poses en el MISMO instante: el más reciente que tengan en común
            stamps = [self.pose(c)[1] for c in ('base_link', 'base_link_ekf', 'base_link_gt')]
            when = min(stamps, key=lambda s: s.nanoseconds)
            o, e, g = (self.pose(c, when)[0] for c in ('base_link', 'base_link_ekf', 'base_link_gt'))
        except Exception:
            return
        self.w.writerow([f'{when.nanoseconds / 1e9 - self.t0:.2f}', *[f'{v:.4f}' for v in (*o, *e, *g)],
                         *[f'{v:.6f}' for v in self.P], self.nl,
                         *([f'{v:.4f}' for v in self.to_map(*self.goal, when)] if self.goal else ['nan'] * 3),
                         *[f'{v:.4f}' for v in self.cmd]])
        if now - self.t0 >= self.dur:
            self.f.close(); raise SystemExit

rclpy.init()
n = Sampler(sys.argv[1], float(sys.argv[2]))
try:
    rclpy.spin(n)
except SystemExit:
    pass
