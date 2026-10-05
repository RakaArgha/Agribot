#!/usr/bin/env python3
"""
sac_controller_node.py  -  Node ROS 2: pose -> SAC -> /mpc/control -> alljoy.py

alljoy.py (mode A / otomatis) membaca /mpc/control sebagai:
    [RPM kiri, RPM kanan, sudut kiri (deg), sudut kanan (deg)]
    sudut relatif: 0 = lurus. alljoy menambah 180 dan membatasi +-31.2 deg.
    Roda belakang dicerminkan otomatis oleh firmware Arduino.

DUA MODE POSE:
    POSE_SOURCE = 'sim'  -> robot virtual di dalam node (uji di rumah, tanpa robot)
    POSE_SOURCE = 'odom' -> baca nav_msgs/Odometry dari topik ODOM_TOPIC (robot asli)

Jalankan (dari ~/bismillah):
    source /opt/ros/jazzy/setup.bash
    python3 sac_controller_node.py
"""
import os, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from sac_policy import SACPolicy, baca_info
from sac_obs import ObservationBuilder
from sac_sil import kinematika_dual

# =========================================================================
#  PENGATURAN  -- bagian yang disesuaikan di farm ditandai [FARM]
# =========================================================================
POSE_SOURCE  = 'sim'      # 'sim' di rumah, 'odom' di robot asli
ODOM_TOPIC   = '/odom'

STEER_SIGN   = +1.0       # [FARM] +1 atau -1. Diisi dari uji arah roda:
                          #   kalau perintah "belok kiri" membuat roda ke KANAN -> -1
RPM_PER_MPS  = 100.0      # [FARM] RPM yang menghasilkan 1 m/s. BELUM DIUKUR.
                          #   Kalibrasi: perintah RPM tetap, ukur jarak/detik.
RPM_SIGN     = +1.0       # [FARM] -1 kalau RPM positif ternyata membuat robot MUNDUR

V_BATAS      = 0.3        # batas kecepatan uji lapangan (m/s), di atas policy
BATAS_KELUAR = 0.8        # berhenti kalau robot > 0.8 m dari path (m)
RADIUS_GOAL  = 0.3        # sama dengan training
# =========================================================================

DEPLOY  = 'deploy'
info    = baca_info(os.path.join(DEPLOY, 'deploy_info.txt'))
L_HALF  = info['L_half']
W       = info['W']
TS      = info['Ts']
RATE    = info['steer_rate_max']
N_GHOST = int(info['n_ghost'])


def ke_roda(v, delta):
    """
    Sudut virtual (konvensi SAC: + = kiri) -> sudut & kecepatan tiap sisi.
    Rumus ICR eksak: jarak roda ke ICR memakai komponen lateral DAN L_HALF.
    Kembali: (deg_kiri, deg_kanan, v_kiri, v_kanan), sudut dlm konvensi SAC.
    """
    if abs(delta) < 1e-4:
        return 0.0, 0.0, v, v
    R = L_HALF / math.tan(delta)                 # >0 belok kiri, <0 belok kanan
    ang_L = math.degrees(math.atan(L_HALF / (R - W/2)))
    ang_R = math.degrees(math.atan(L_HALF / (R + W/2)))
    v_L = v * math.hypot(R - W/2, L_HALF) / abs(R)
    v_R = v * math.hypot(R + W/2, L_HALF) / abs(R)
    return ang_L, ang_R, v_L, v_R


def yaw_dari_quat(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class SACController(Node):
    def __init__(self):
        super().__init__('sac_controller')
        self.pub = self.create_publisher(Float64MultiArray, '/mpc/control', 10)

        self.pol = SACPolicy()
        self.ob  = ObservationBuilder()
        P = self.ob.P
        self.N_real = self.ob.N - N_GHOST
        self.goal = P[self.N_real-1]
        self.path = P[:self.N_real]

        self.pose = None                  # (x, y, th) dalam frame start
        self.odom0 = None                 # pose odometri saat start
        if POSE_SOURCE == 'sim':
            self.pose = (0.0, 0.0, 0.0)
        else:
            self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 10)

        self.delta_prev = 0.0
        self.selesai = False
        self.t0 = time.time()
        self.log = open(time.strftime('run_%Y%m%d_%H%M%S.csv'), 'w')
        self.log.write('t,x,y,th,v,delta,cte,rpm_L,rpm_R,deg_L,deg_R\n')

        self.create_timer(TS, self.tick)
        self.get_logger().info(
            f"SAC controller | pose={POSE_SOURCE} | V_BATAS={V_BATAS} m/s | "
            f"STEER_SIGN={STEER_SIGN:+.0f} | RPM_PER_MPS={RPM_PER_MPS} | "
            f"tujuan=({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def cb_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = yaw_dari_quat(msg.pose.pose.orientation)
        if self.odom0 is None:                    # titik start = (0,0,0)
            self.odom0 = (x, y, th)
            self.get_logger().info("Odometri pertama diterima -> dijadikan titik start")
        x0, y0, th0 = self.odom0
        dx, dy = x - x0, y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        rel_th = math.atan2(math.sin(th - th0), math.cos(th - th0))
        self.pose = (c*dx - s*dy, s*dx + c*dy, rel_th)

    def kirim(self, rpm_L, rpm_R, deg_L, deg_R):
        m = Float64MultiArray()
        m.data = [float(rpm_L), float(rpm_R), float(deg_L), float(deg_R)]
        self.pub.publish(m)

    def berhenti(self, alasan):
        if not self.selesai:
            self.get_logger().warn(f"BERHENTI: {alasan}")
        self.selesai = True
        self.kirim(0.0, 0.0, 0.0, 0.0)

    def tick(self):
        if self.selesai:
            self.kirim(0.0, 0.0, 0.0, 0.0)
            return
        if self.pose is None:                     # menunggu odometri
            self.kirim(0.0, 0.0, 0.0, 0.0)
            return

        x, y, th = self.pose
        cte = float(np.min(np.hypot(self.path[:, 0]-x, self.path[:, 1]-y)))
        if math.hypot(x-self.goal[0], y-self.goal[1]) < RADIUS_GOAL:
            return self.berhenti(f"SAMPAI TUJUAN (t={time.time()-self.t0:.1f} s)")
        if cte > BATAS_KELUAR:
            return self.berhenti(f"KELUAR JALUR (CTE {cte:.2f} m)")

        obs = self.ob(x, y, th)
        v, delta_cmd = self.pol.act(obs)

        step = RATE * TS                          # Rate Limiter, sama dg Simulink
        delta = min(max(delta_cmd, self.delta_prev - step), self.delta_prev + step)
        self.delta_prev = delta
        v = min(v, V_BATAS)

        deg_L, deg_R, v_L, v_R = ke_roda(v, delta)
        rpm_L = RPM_SIGN * v_L * RPM_PER_MPS
        rpm_R = RPM_SIGN * v_R * RPM_PER_MPS
        out_L, out_R = STEER_SIGN * deg_L, STEER_SIGN * deg_R
        self.kirim(rpm_L, rpm_R, out_L, out_R)

        t = time.time() - self.t0
        self.log.write(f"{t:.2f},{x:.4f},{y:.4f},{th:.4f},{v:.4f},{delta:.4f},"
                       f"{cte:.4f},{rpm_L:.1f},{rpm_R:.1f},{out_L:.2f},{out_R:.2f}\n")

        if POSE_SOURCE == 'sim':                  # robot virtual maju satu langkah
            self.pose = kinematika_dual(v, delta, -delta, x, y, th)

    def tutup(self):
        for _ in range(5):                        # pastikan perintah terakhir = diam
            self.kirim(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.02)
        self.log.close()


def main():
    rclpy.init()
    node = SACController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tutup()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
sac_controller_node.py  -  Node ROS 2: pose -> SAC -> /mpc/control -> alljoy.py

alljoy.py (mode A / otomatis) membaca /mpc/control sebagai:
    [RPM kiri, RPM kanan, sudut kiri (deg), sudut kanan (deg)]
    sudut relatif: 0 = lurus. alljoy menambah 180 dan membatasi +-31.2 deg.
    Roda belakang dicerminkan otomatis oleh firmware Arduino.

DUA MODE POSE:
    POSE_SOURCE = 'sim'  -> robot virtual di dalam node (uji di rumah, tanpa robot)
    POSE_SOURCE = 'odom' -> baca nav_msgs/Odometry dari topik ODOM_TOPIC (robot asli)

Jalankan (dari ~/bismillah):
    source /opt/ros/jazzy/setup.bash
    python3 sac_controller_node.py
"""
import os, math, time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from sac_policy import SACPolicy, baca_info
from sac_obs import ObservationBuilder
from sac_sil import kinematika_dual

# =========================================================================
#  PENGATURAN  -- bagian yang disesuaikan di farm ditandai [FARM]
# =========================================================================
POSE_SOURCE  = 'sim'      # 'sim' di rumah, 'odom' di robot asli
ODOM_TOPIC   = '/odom'

STEER_SIGN   = +1.0       # [FARM] +1 atau -1. Diisi dari uji arah roda:
                          #   kalau perintah "belok kiri" membuat roda ke KANAN -> -1
RPM_PER_MPS  = 100.0      # [FARM] RPM yang menghasilkan 1 m/s. BELUM DIUKUR.
                          #   Kalibrasi: perintah RPM tetap, ukur jarak/detik.
RPM_SIGN     = +1.0       # [FARM] -1 kalau RPM positif ternyata membuat robot MUNDUR

V_BATAS      = 0.3        # batas kecepatan uji lapangan (m/s), di atas policy
BATAS_KELUAR = 0.8        # berhenti kalau robot > 0.8 m dari path (m)
RADIUS_GOAL  = 0.3        # sama dengan training
# =========================================================================

DEPLOY  = 'deploy'
info    = baca_info(os.path.join(DEPLOY, 'deploy_info.txt'))
L_HALF  = info['L_half']
W       = info['W']
TS      = info['Ts']
RATE    = info['steer_rate_max']
N_GHOST = int(info['n_ghost'])


def ke_roda(v, delta):
    """
    Sudut virtual (konvensi SAC: + = kiri) -> sudut & kecepatan tiap sisi.
    Rumus ICR eksak: jarak roda ke ICR memakai komponen lateral DAN L_HALF.
    Kembali: (deg_kiri, deg_kanan, v_kiri, v_kanan), sudut dlm konvensi SAC.
    """
    if abs(delta) < 1e-4:
        return 0.0, 0.0, v, v
    R = L_HALF / math.tan(delta)                 # >0 belok kiri, <0 belok kanan
    ang_L = math.degrees(math.atan(L_HALF / (R - W/2)))
    ang_R = math.degrees(math.atan(L_HALF / (R + W/2)))
    v_L = v * math.hypot(R - W/2, L_HALF) / abs(R)
    v_R = v * math.hypot(R + W/2, L_HALF) / abs(R)
    return ang_L, ang_R, v_L, v_R


def yaw_dari_quat(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class SACController(Node):
    def __init__(self):
        super().__init__('sac_controller')
        self.pub = self.create_publisher(Float64MultiArray, '/mpc/control', 10)

        self.pol = SACPolicy()
        self.ob  = ObservationBuilder()
        P = self.ob.P
        self.N_real = self.ob.N - N_GHOST
        self.goal = P[self.N_real-1]
        self.path = P[:self.N_real]

        self.pose = None                  # (x, y, th) dalam frame start
        self.odom0 = None                 # pose odometri saat start
        if POSE_SOURCE == 'sim':
            self.pose = (0.0, 0.0, 0.0)
        else:
            self.create_subscription(Odometry, ODOM_TOPIC, self.cb_odom, 10)

        self.delta_prev = 0.0
        self.selesai = False
        self.t0 = time.time()
        self.log = open(time.strftime('run_%Y%m%d_%H%M%S.csv'), 'w')
        self.log.write('t,x,y,th,v,delta,cte,rpm_L,rpm_R,deg_L,deg_R\n')

        self.create_timer(TS, self.tick)
        self.get_logger().info(
            f"SAC controller | pose={POSE_SOURCE} | V_BATAS={V_BATAS} m/s | "
            f"STEER_SIGN={STEER_SIGN:+.0f} | RPM_PER_MPS={RPM_PER_MPS} | "
            f"tujuan=({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def cb_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        th = yaw_dari_quat(msg.pose.pose.orientation)
        if self.odom0 is None:                    # titik start = (0,0,0)
            self.odom0 = (x, y, th)
            self.get_logger().info("Odometri pertama diterima -> dijadikan titik start")
        x0, y0, th0 = self.odom0
        dx, dy = x - x0, y - y0
        c, s = math.cos(-th0), math.sin(-th0)
        rel_th = math.atan2(math.sin(th - th0), math.cos(th - th0))
        self.pose = (c*dx - s*dy, s*dx + c*dy, rel_th)

    def kirim(self, rpm_L, rpm_R, deg_L, deg_R):
        m = Float64MultiArray()
        m.data = [float(rpm_L), float(rpm_R), float(deg_L), float(deg_R)]
        self.pub.publish(m)

    def berhenti(self, alasan):
        if not self.selesai:
            self.get_logger().warn(f"BERHENTI: {alasan}")
        self.selesai = True
        self.kirim(0.0, 0.0, 0.0, 0.0)

    def tick(self):
        if self.selesai:
            self.kirim(0.0, 0.0, 0.0, 0.0)
            return
        if self.pose is None:                     # menunggu odometri
            self.kirim(0.0, 0.0, 0.0, 0.0)
            return

        x, y, th = self.pose
        cte = float(np.min(np.hypot(self.path[:, 0]-x, self.path[:, 1]-y)))
        if math.hypot(x-self.goal[0], y-self.goal[1]) < RADIUS_GOAL:
            return self.berhenti(f"SAMPAI TUJUAN (t={time.time()-self.t0:.1f} s)")
        if cte > BATAS_KELUAR:
            return self.berhenti(f"KELUAR JALUR (CTE {cte:.2f} m)")

        obs = self.ob(x, y, th)
        v, delta_cmd = self.pol.act(obs)

        step = RATE * TS                          # Rate Limiter, sama dg Simulink
        delta = min(max(delta_cmd, self.delta_prev - step), self.delta_prev + step)
        self.delta_prev = delta
        v = min(v, V_BATAS)

        deg_L, deg_R, v_L, v_R = ke_roda(v, delta)
        rpm_L = RPM_SIGN * v_L * RPM_PER_MPS
        rpm_R = RPM_SIGN * v_R * RPM_PER_MPS
        out_L, out_R = STEER_SIGN * deg_L, STEER_SIGN * deg_R
        self.kirim(rpm_L, rpm_R, out_L, out_R)

        t = time.time() - self.t0
        self.log.write(f"{t:.2f},{x:.4f},{y:.4f},{th:.4f},{v:.4f},{delta:.4f},"
                       f"{cte:.4f},{rpm_L:.1f},{rpm_R:.1f},{out_L:.2f},{out_R:.2f}\n")

        if POSE_SOURCE == 'sim':                  # robot virtual maju satu langkah
            self.pose = kinematika_dual(v, delta, -delta, x, y, th)

    def tutup(self):
        for _ in range(5):                        # pastikan perintah terakhir = diam
            self.kirim(0.0, 0.0, 0.0, 0.0)
            time.sleep(0.02)
        self.log.close()


def main():
    rclpy.init()
    node = SACController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tutup()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()