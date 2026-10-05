# alljoy_safe.py

import pygame
import can
import time
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# =============================================================================
# alljoy_safe.py = alljoy.py + 2 perbaikan keselamatan (ditandai [SAFETY]):
#   1. WATCHDOG: di mode otomatis, kalau /mpc/control tidak datang selama
#      AUTO_TIMEOUT detik (node crash / laptop hang), RPM dipaksa 0.
#   2. TOMBOL Y sekarang juga menghentikan MOTOR PENGGERAK. Dulu Y hanya
#      mematikan motor kemudi, sehingga robot bisa tetap melaju tanpa kemudi.
# Selain itu, perilaku identik dengan alljoy.py asli.
# =============================================================================

# --- 1. 車體硬體參數 ---
L_HALF = 0.86           # 半軸長 (m)
W = 1.65                # 輪距 (m)
CENTER_VAL = 180.0      # 轉向中心值
STEER_LIMIT_DEG = 31.2  # 車體轉向極限

AUTO_TIMEOUT = 0.3      # [SAFETY] detik tanpa pesan /mpc/control -> berhenti

# --- 2. 系統狀態變數 ---
current_mode = 0        # 0:預設, 1:手動(JOY), 2:自動(MATLAB)
motor_enabled = False
heartbeat = 0x01

# 手動模式指令暫存
joy_angle = 180.0
joy_rpm = 0.0

# 自動模式指令暫存
auto_left_rpm = 0.0
auto_right_rpm = 0.0
auto_left_angle = 180.0
auto_right_angle = 180.0
auto_msg_received = False
auto_last_time = 0.0    # [SAFETY] waktu pesan /mpc/control terakhir
watchdog_aktif = False  # [SAFETY] untuk tampilan monitor

# thread lock
auto_lock = threading.Lock()


# --- 3. 核心運算函式 ---
def calculate_checksum(data_list):
    """計算前 6 bytes 的總和作為校驗碼"""
    checksum = sum(data_list[:6])
    return (checksum >> 8) & 0xFF, checksum & 0xFF


def pack_motor_data(speed, hb):
    """封裝馬達速度 CAN 數據"""
    direction = 0x8000 if speed < 0 else 0x4000
    cmd = direction | (int(abs(speed)) & 0x3FFF)
    pre = [(cmd >> 8) & 0xFF, cmd & 0xFF, 0, 0, 0, hb]
    cs_h, cs_l = calculate_checksum(pre)
    return pre + [cs_h, cs_l]


def get_ackermann_logic(steer_angle, base_rpm):
    """手動模式阿克曼幾何：計算各輪角度與速差"""
    delta_rad = math.radians(CENTER_VAL - steer_angle)

    if abs(delta_rad) < 0.005:  # 直行模式
        return CENTER_VAL, CENTER_VAL, base_rpm, base_rpm

    R = L_HALF / math.tan(delta_rad)
    angle_L = CENTER_VAL - math.degrees(math.atan(L_HALF / (R + W / 2)))
    angle_R = CENTER_VAL - math.degrees(math.atan(L_HALF / (R - W / 2)))
    v_L = base_rpm * (abs(R + W / 2) / abs(R))
    v_R = base_rpm * (abs(R - W / 2) / abs(R))
    return angle_L, angle_R, v_L, v_R


def clamp_steer_angle(angle_deg):
    """限制轉向角只能在 180 ± 31.2 度"""
    min_angle = CENTER_VAL - STEER_LIMIT_DEG
    max_angle = CENTER_VAL + STEER_LIMIT_DEG
    return max(min_angle, min(max_angle, angle_deg))


# --- 4. ROS2 subscriber for /mpc/control ---
class MPCControlSubscriber(Node):
    def __init__(self):
        super().__init__('mpc_control_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/mpc/control',
            self.control_callback,
            10
        )

    def control_callback(self, msg):
        global auto_left_rpm, auto_right_rpm
        global auto_left_angle, auto_right_angle
        global auto_msg_received, auto_last_time

        data = list(msg.data)
        if len(data) < 4:
            return

        # /mpc/control = [左輪速RPM, 右輪速RPM, 左角度deg, 右角度deg]
        left_rpm_cmd = float(data[0])
        right_rpm_cmd = float(data[1])
        left_deg_cmd = float(data[2])
        right_deg_cmd = float(data[3])

        # MPC 前方=0度 -> 控制板中心=180度
        left_target = left_deg_cmd + CENTER_VAL
        right_target = right_deg_cmd + CENTER_VAL

        # 安全限制
        left_target = clamp_steer_angle(left_target)
        right_target = clamp_steer_angle(right_target)

        with auto_lock:
            auto_left_rpm = left_rpm_cmd
            auto_right_rpm = right_rpm_cmd
            auto_left_angle = left_target
            auto_right_angle = right_target
            auto_msg_received = True
            auto_last_time = time.time()      # [SAFETY]


def ros2_spin_thread():
    rclpy.init()
    node = MPCControlSubscriber()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --- 5. 初始化硬體 ---
pygame.init()
try:
    joy = pygame.joystick.Joystick(0)
    joy.init()
except:
    print("找不到搖桿！")
    sys.exit()

try:
    # CH0: 動力馬達 | CH1: 轉向 Arduino
    bus0 = can.interface.Bus(interface='kvaser', channel=0, bitrate=250000)
    bus1 = can.interface.Bus(interface='kvaser', channel=1, bitrate=250000)
    print("=" * 60)
    print("系統啟動成功 | 雙通道連線正常  [versi SAFE: watchdog + Y stop]")
    print("按鍵分配: [B: 手動模式] [A: 自動模式] [Y: 電源開關] [Start: 歸零]")
    print("=" * 60)
except Exception as e:
    print(f"CAN 初始化失敗: {e}")
    sys.exit()

# 啟動 ROS2 背景執行緒
ros_thread = threading.Thread(target=ros2_spin_thread, daemon=True)
ros_thread.start()


# --- 6. 主迴圈 ---
try:
    while True:
        # --- A. 事件偵測與模式控制 ---
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 2:  # B鍵 -> 手動
                    current_mode = 1
                    print("\n>> 切換至模式 1: 手動控制")

                elif event.button == 1:  # A鍵 -> 自動
                    current_mode = 2
                    with auto_lock:
                        auto_msg_received = False
                        auto_last_time = 0.0          # [SAFETY]
                        auto_left_rpm = 0.0
                        auto_right_rpm = 0.0
                        auto_left_angle = CENTER_VAL
                        auto_right_angle = CENTER_VAL
                    print("\n>> 切換至模式 2: 自動控制 (MATLAB)，等待 MPC 指令中...")

                elif event.button == 3:  # Y鍵 -> 電源
                    motor_enabled = not motor_enabled

                elif event.button == 9:  # Start鍵 -> 歸零
                    joy_angle = CENTER_VAL
                    joy_rpm = 0.0

            if event.type == pygame.JOYHATMOTION:
                if current_mode == 1:
                    hx, hy = event.value
                    joy_angle += hx * -2.5
                    joy_rpm += hy * 5.0

        # --- B. 仲裁邏輯 ---
        watchdog_aktif = False
        if current_mode == 1:
            # 手動模式：沿用原本阿克曼邏輯
            t_angle = clamp_steer_angle(joy_angle)
            lf_ang, rf_ang, v_L, v_R = get_ackermann_logic(t_angle, joy_rpm)

        elif current_mode == 2:
            # 自動模式：若尚未收到 MPC 指令，先停車回正
            with auto_lock:
                got_auto_cmd = auto_msg_received
                last_time = auto_last_time
                lf_auto = auto_left_angle
                rf_auto = auto_right_angle
                rpm_left_auto = auto_left_rpm
                rpm_right_auto = auto_right_rpm

            # [SAFETY] pesan terlalu lama tidak datang -> anggap controller mati
            basi = (time.time() - last_time) > AUTO_TIMEOUT

            if not got_auto_cmd or basi:
                watchdog_aktif = got_auto_cmd and basi       # [SAFETY]
                lf_ang = CENTER_VAL
                rf_ang = CENTER_VAL
                v_L = 0.0
                v_R = 0.0
            else:
                lf_ang = clamp_steer_angle(lf_auto)
                rf_ang = clamp_steer_angle(rf_auto)
                v_L = rpm_left_auto
                v_R = rpm_right_auto

        else:
            # 預設等待模式
            lf_ang = CENTER_VAL
            rf_ang = CENTER_VAL
            v_L = 0.0
            v_R = 0.0

        # [SAFETY] Y = OFF sekarang juga menghentikan motor penggerak
        if not motor_enabled:
            v_L = 0.0
            v_R = 0.0

        # --- C. 封裝 CAN ---
        en_byte = 0x01 if motor_enabled else 0x00

        lf_s = int(round(lf_ang * 10))
        rf_s = int(round(rf_ang * 10))
        angle_data = [
            (lf_s >> 8) & 0xFF, lf_s & 0xFF,
            (rf_s >> 8) & 0xFF, rf_s & 0xFF,
            0, 0, 0, en_byte
        ]

        motor_data_L = pack_motor_data(v_L, heartbeat)
        motor_data_R = pack_motor_data(v_R, heartbeat)

        # --- D. 發送數據 ---
        try:
            # 轉向封包：ID 跟手動相同
            bus1.send(can.Message(arbitration_id=0x123, data=angle_data, is_extended_id=False))

            # 左前 = 左後
            bus0.send(can.Message(arbitration_id=0x18705501, data=motor_data_L, is_extended_id=True))
            bus0.send(can.Message(arbitration_id=0x18705503, data=motor_data_L, is_extended_id=True))

            # 右前 = 右後
            bus0.send(can.Message(arbitration_id=0x18705502, data=motor_data_R, is_extended_id=True))
            bus0.send(can.Message(arbitration_id=0x18705504, data=motor_data_R, is_extended_id=True))
        except:
            pass

        # --- E. 監控介面 ---
        m_str = {0: "等待", 1: "手動", 2: "自動"}[current_mode]
        p_str = "[ON]" if motor_enabled else "[OFF]"
        w_str = " WATCHDOG-STOP" if watchdog_aktif else ""       # [SAFETY]
        sys.stdout.write(
            f"\r模式:{m_str:4s} {p_str} | 左角:{lf_ang:6.1f} 右角:{rf_ang:6.1f} | 左RPM:{v_L:6.1f} 右RPM:{v_R:6.1f}{w_str}   "
        )
        sys.stdout.flush()

        heartbeat = (heartbeat + 1) % 0xFF or 0x01
        time.sleep(0.05)

except KeyboardInterrupt:
    pygame.quit()
    print("\n系統已安全關閉")
