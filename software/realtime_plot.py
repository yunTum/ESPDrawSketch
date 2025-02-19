import socket
import struct
import time
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import pandas as pd

class RealTimeAnalyzer:
    def __init__(self):
        # UDP設定
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 4215
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        self.sock.settimeout(0.01)  # タイムアウトを10msに設定
        print(f"UDPリスニング開始: {self.UDP_IP}:{self.UDP_PORT}")

        # データ保存用の設定
        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f"acceleration_data_{current_time}.csv"
        with open(self.csv_filename, 'w', newline='') as f:
            f.write("timestamp,sequence,acc_x,acc_y,acc_z\n")

        # データバッファの初期化（最新の100点を保持）
        self.buffer_size = 100
        self.times = deque(maxlen=self.buffer_size)
        self.acc_x = deque(maxlen=self.buffer_size)
        self.acc_y = deque(maxlen=self.buffer_size)
        self.acc_z = deque(maxlen=self.buffer_size)
        self.vel_x = deque(maxlen=self.buffer_size)
        self.vel_y = deque(maxlen=self.buffer_size)
        self.vel_z = deque(maxlen=self.buffer_size)
        self.pos_x = deque(maxlen=self.buffer_size)
        self.pos_y = deque(maxlen=self.buffer_size)
        self.pos_z = deque(maxlen=self.buffer_size)
        self.gyro_x = deque(maxlen=self.buffer_size)
        self.gyro_y = deque(maxlen=self.buffer_size)
        self.gyro_z = deque(maxlen=self.buffer_size)
        self.roll = deque(maxlen=self.buffer_size)
        self.pitch = deque(maxlen=self.buffer_size)
        self.yaw = deque(maxlen=self.buffer_size)

        # 相補フィルタの係数
        self.alpha = 0.96  # ジャイロセンサーの重み
        self.dt = 0.05    # サンプリング周期（50ms）

        # グラフの初期化
        self.fig = plt.figure(figsize=(12, 8))
        self.ax1 = self.fig.add_subplot(311)
        self.ax2 = self.fig.add_subplot(312)
        self.ax3 = self.fig.add_subplot(313)
        # self.ax4 = self.fig.add_subplot(414)

        # 3Dグラフの初期化
        self.fig3d = plt.figure(figsize=(8, 8))
        self.ax3d = self.fig3d.add_subplot(111, projection='3d')

        # アニメーションの開始
        self.ani = FuncAnimation(self.fig, self.update_graph, interval=30, cache_frame_data=False)
        self.ani3d = FuncAnimation(self.fig3d, self.update_3d_graph, interval=30, cache_frame_data=False)

        # 閾値設定
        self.acc_threshold = 0.03  # 加速度の閾値 (m/s²)
        self.vel_threshold = 0.05  # 速度の閾値 (m/s)
        self.damping = 0.85  # 速度の減衰係数（1未満）

        # 前回のデータ
        self.last_seq = None
        self.last_update_time = None

    def decode_float_signed(self, data):
        sign = data[0]
        value = (data[1] << 8) | data[2]
        return -value / 100.0 if sign else value / 100.0

    def update_data(self):
        try:
            data, addr = self.sock.recvfrom(21)
            if len(data) != 21 or data[0] != 0x55:
                return

            # データの解析
            seq_no = (data[1] << 8) | data[2]
            if self.last_seq is not None and seq_no <= self.last_seq:
                return  # 古いデータや重複データは無視
            self.last_seq = seq_no
            # 現在時刻の確認
            current_time = time.time()
            if self.last_update_time is not None:
                if current_time - self.last_update_time < 0.01:  # 10ms以内のデータは無視
                    return
            self.last_update_time = current_time
            acc_x = self.decode_float_signed(data[3:6])
            acc_y = self.decode_float_signed(data[6:9])
            acc_z = self.decode_float_signed(data[9:12])

            gyro_x = self.decode_float_signed(data[12:15])
            gyro_y = self.decode_float_signed(data[15:18])
            gyro_z = self.decode_float_signed(data[18:21])

            # 加速度の閾値処理
            acc_x = 0 if abs(acc_x) < self.acc_threshold else acc_x
            acc_y = 0 if abs(acc_y) < self.acc_threshold else acc_y
            acc_z = 0 if abs(acc_z) < self.acc_threshold else acc_z

            # print(f"acc_x: {acc_x}, acc_y: {acc_y}, acc_z: {acc_z}")
            
            # タイムスタンプの記録
            timestamp = datetime.now()
            current_time = (timestamp - self.start_time).total_seconds() if hasattr(self, 'start_time') else 0
            if not hasattr(self, 'start_time'):
                self.start_time = timestamp

            # CSVに記録
            with open(self.csv_filename, 'a', newline='') as f:
                f.write(f"{timestamp.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]},{seq_no},{acc_x:.2f},{acc_y:.2f},{acc_z:.2f}\n")

            # データの追加
            self.times.append(current_time)
            self.acc_x.append(acc_x)
            self.acc_y.append(acc_y)
            self.acc_z.append(acc_z)

            # 姿勢角の計算（相補フィルタ）
            if len(self.times) > 0:
                # 加速度からの姿勢角計算
                roll_acc = np.arctan2(acc_y, acc_z) * 180 / np.pi
                pitch_acc = np.arctan2(-acc_x, np.sqrt(acc_y**2 + acc_z**2)) * 180 / np.pi

                # 前回の姿勢角度を取得
                last_roll = self.roll[-1] if self.roll else roll_acc
                last_pitch = self.pitch[-1] if self.pitch else pitch_acc
                last_yaw = self.yaw[-1] if self.yaw else 0

                # 相補フィルタでジャイロと加速度を統合
                roll_gyro = last_roll + gyro_x * self.dt
                pitch_gyro = last_pitch + gyro_y * self.dt
                yaw = last_yaw + gyro_z * self.dt

                # 相補フィルタの適用
                roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_acc
                pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_acc

                self.roll.append(roll)
                self.pitch.append(pitch)
                self.yaw.append(yaw)

                # 姿勢角を考慮した加速度の補正
                cos_roll = np.cos(roll * np.pi / 180)
                sin_roll = np.sin(roll * np.pi / 180)
                cos_pitch = np.cos(pitch * np.pi / 180)
                sin_pitch = np.sin(pitch * np.pi / 180)

                # 重力の影響を除去した加速度の計算
                acc_x_corrected = acc_x - sin_pitch
                acc_y_corrected = acc_y + sin_roll * cos_pitch
                acc_z_corrected = acc_z + cos_roll * cos_pitch

                # 補正後の加速度を使用して速度と位置を計算
            # 速度と位置の計算
            if len(self.times) > 1:
                dt = self.times[-1] - self.times[-2]
                
                # 速度の計算（補正後の加速度を使用）
                last_vel_x = self.vel_x[-1] if self.vel_x else 0
                last_vel_y = self.vel_y[-1] if self.vel_y else 0
                last_vel_z = self.vel_z[-1] if self.vel_z else 0

                new_vel_x = last_vel_x + acc_x_corrected * dt
                new_vel_y = last_vel_y + acc_y_corrected * dt
                new_vel_z = last_vel_z + acc_z_corrected * dt

                # 速度の減衰処理
                if abs(acc_x_corrected) < self.acc_threshold:
                    new_vel_x *= self.damping
                if abs(acc_y_corrected) < self.acc_threshold:
                    new_vel_y *= self.damping
                if abs(acc_z_corrected) < self.acc_threshold:
                    new_vel_z *= self.damping

                # 速度の閾値処理
                new_vel_x = 0 if abs(new_vel_x) < self.vel_threshold else new_vel_x
                new_vel_y = 0 if abs(new_vel_y) < self.vel_threshold else new_vel_y
                new_vel_z = 0 if abs(new_vel_z) < self.vel_threshold else new_vel_z
            else:
                new_vel_x = 0
                new_vel_y = 0
                new_vel_z = 0

            # 速度データの追加
            self.vel_x.append(new_vel_x)
            self.vel_y.append(new_vel_y)
            self.vel_z.append(new_vel_z)

            # 位置の計算
            if len(self.times) > 1:
                dt = self.times[-1] - self.times[-2]
                last_pos_x = self.pos_x[-1] if self.pos_x else 0
                last_pos_y = self.pos_y[-1] if self.pos_y else 0
                last_pos_z = self.pos_z[-1] if self.pos_z else 0
                
                self.pos_x.append(last_pos_x + new_vel_x * dt)
                self.pos_y.append(last_pos_y + new_vel_y * dt)
                self.pos_z.append(last_pos_z + new_vel_z * dt)
            else:
                self.pos_x.append(0)
                self.pos_y.append(0)
                self.pos_z.append(0)
        except socket.timeout:
            # タイムアウトの場合は何もしない
            pass
        except Exception as e:
            print(f"エラー発生: {e}")

    def update_graph(self, frame):
        for _ in range(5):  # 最大5回試行
            try:
                self.update_data()
                break
            except socket.timeout:
                pass
        
        # グラフのクリア
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        # 加速度のプロット
        self.ax1.plot(list(self.times), list(self.acc_x), label='X')
        self.ax1.plot(list(self.times), list(self.acc_y), label='Y')
        self.ax1.plot(list(self.times), list(self.acc_z), label='Z')
        self.ax1.set_title('ACC X,Y,Z')
        self.ax1.set_ylabel('ACC (m/s²)')
        self.ax1.grid(True)
        self.ax1.legend()

        # 速度のプロット
        self.ax2.plot(list(self.times), list(self.vel_x), label='X')
        self.ax2.plot(list(self.times), list(self.vel_y), label='Y')
        self.ax2.plot(list(self.times), list(self.vel_z), label='Z')
        self.ax2.set_title('VEL X,Y,Z')
        self.ax2.set_ylabel('VEL (m/s)')
        self.ax2.grid(True)
        self.ax2.legend()

        # 位置のプロット
        self.ax3.plot(list(self.times), list(self.pos_x), label='X')
        self.ax3.plot(list(self.times), list(self.pos_y), label='Y')
        self.ax3.plot(list(self.times), list(self.pos_z), label='Z')
        self.ax3.set_title('POS X,Y,Z')
        self.ax3.set_xlabel('TIME (sec)')
        self.ax3.set_ylabel('POS (cm)')
        self.ax3.grid(True)
        self.ax3.legend()

        
        # 姿勢角のプロット追加
        # self.ax4.plot(list(self.times), list(self.roll), label='Roll')
        # self.ax4.plot(list(self.times), list(self.pitch), label='Pitch')
        # self.ax4.plot(list(self.times), list(self.yaw), label='Yaw')
        # self.ax4.set_title('Attitude')
        # self.ax4.set_xlabel('TIME (sec)')
        # self.ax4.set_ylabel('Angle (deg)')
        # self.ax4.grid(True)
        # self.ax4.legend()

        plt.tight_layout()

    def update_3d_graph(self, frame):
        # 3Dグラフのクリア
        self.ax3d.clear()
        
        # 3D軌跡のプロット
        self.ax3d.plot(list(self.pos_x), list(self.pos_y), list(self.pos_z))
        self.ax3d.set_title('3D TRAJECTORY')
        self.ax3d.set_xlabel('X (cm)')
        self.ax3d.set_ylabel('Y (cm)')
        self.ax3d.set_zlabel('Z (cm)')

    def run(self):
        plt.show()

if __name__ == "__main__":
    analyzer = RealTimeAnalyzer()
    analyzer.run()