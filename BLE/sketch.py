import asyncio
import websockets
import json
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time

class KalmanFilter:
    def __init__(self):
        # 状態ベクトル [x, y, vx, vy]
        self.state = np.zeros(4)
        # 共分散行列
        self.P = np.eye(4) * 0.1
        # プロセスノイズ
        self.Q = np.eye(4)
        self.Q[0:2, 0:2] *= 0.01  # 位置のプロセスノイズ
        self.Q[2:4, 2:4] *= 0.05  # 速度のプロセスノイズ
        # 観測ノイズ
        self.R = np.eye(2) * 0.1
        # 時間間隔
        self.dt = 0.1

    def predict(self, acc_x, acc_y):
        # 状態遷移行列
        F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # 制御入力行列
        B = np.array([
            [0.5 * self.dt**2, 0],
            [0, 0.5 * self.dt**2],
            [self.dt, 0],
            [0, self.dt]
        ])
        
        # 加速度による制御入力
        u = np.array([acc_x, acc_y])
        
        # 状態予測
        self.state = F @ self.state + B @ u
        # 共分散予測
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        if np.any(np.isnan(measurement)):
            return
            
        # 観測行列 (位置のみ観測)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # カルマンゲイン
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # 状態更新
        y = measurement - H @ self.state
        self.state = self.state + K @ y
        
        # 共分散更新
        self.P = (np.eye(4) - K @ H) @ self.P

class PositionTracker:
    def __init__(self):
        # self.kalman_filter = KalmanFilter()
        self.last_position = np.array([0.0, 0.0, 0.0])  # 初期位置
        self.last_velocity = np.array([0.0, 0.0, 0.0])  # 初期速度
        self.dt = 0.1  # サンプリング時間（秒）
        # デックを使用してデータ履歴を管理（最大500点）
        self.position_history = deque(maxlen=500)

        self.plot_lim = 10

        self.last_update = time.time()
        self.setup_plot()

        # センサーデータの初期値を設定
        self.last_acc_x = 0
        self.last_acc_y = 0
        self.last_acc_z = 0
        self.last_gyro_x = 0
        self.last_gyro_y = 0
        self.last_gyro_z = 0
        self.moving_forward = False
        self.heading = 0  # 進行方向（度）
        self.acc_magnitude = 0
        self.movement_direction_threshold = 20  # 前方向から±45度以内を許容
        self.acc_x_buffer = deque(maxlen=50)
        self.acc_y_buffer = deque(maxlen=50)
        self.acc_z_buffer = deque(maxlen=50)
        self.gyro_x_buffer = deque(maxlen=50)
        self.gyro_y_buffer = deque(maxlen=50)
        self.gyro_z_buffer = deque(maxlen=50)
        self.roll_buffer = deque(maxlen=50)
        self.pitch_buffer = deque(maxlen=50)
        self.yaw_buffer = deque(maxlen=50)

        self.get_imu_count = 0

        self.acc_data_threshold = 0.4
        self.acc_magnitude_threshold = 0.008 #0.01
        self.gyro_data_threshold = 0.1

    def move_imu_data(self, acc_x, acc_y, yaw):
        # 加速度データをバッファに追加
        self.acc_x_buffer.append(acc_x)
        self.acc_y_buffer.append(acc_y)
        
        # 加速度の移動平均を計算
        avg_acc_x = sum(self.acc_x_buffer) / len(self.acc_x_buffer)
        avg_acc_y = sum(self.acc_y_buffer) / len(self.acc_y_buffer)
        
        # 加速度の大きさを計算
        acc_magnitude = np.sqrt(avg_acc_x**2 + avg_acc_y**2)
        self.acc_magnitude = acc_magnitude

        # 加速度が閾値を超えた場合のみ移動
        if acc_magnitude > self.acc_magnitude_threshold:
            # 加速度の方向を計算
            # self.heading = np.degrees(np.arctan2(avg_acc_y, avg_acc_x))
            self.heading = yaw
            
            # 加速度の大きさに基づいて移動量を決定（スケーリング係数で調整）
            scale_factor = 0.8  # この値を調整して移動量を制御
            movement_distance = scale_factor * acc_magnitude
            
            # 方向に基づいて x, y 成分に分解
            dx = movement_distance * np.cos(np.radians(self.heading))
            dy = movement_distance * np.sin(np.radians(self.heading))
            print(f"dx: {dx}, dy: {dy}")
            print(f"heading: {self.heading}")
            print(f"acc_x: {acc_x}, acc_y: {acc_y}")
            
            # 現在位置の更新
            self.last_position[0] += dx
            self.last_position[1] += dy
            
            # 新しい位置を履歴に追加
            new_position = np.array([self.last_position[0], self.last_position[1]])
            self.position_history.append(new_position)
            
            self.moving_forward = True
        else:
            self.moving_forward = False
        
    def setup_plot(self):
        plt.ion()  # インタラクティブモードを有効化
        # self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig, (self.ax, self.text_ax) = plt.subplots(2, 1, figsize=(8, 10), 
                                                    gridspec_kw={'height_ratios': [4, 1]})
        self.ax.set_xlim(-self.plot_lim, self.plot_lim)
        self.ax.set_ylim(-self.plot_lim, self.plot_lim)
        self.ax.grid(True)
        
        # 現在位置のプロット
        self.scatter = self.ax.scatter([], [], c='r', s=100, label='Current Position')

        # 向きを示す矢印
        self.direction_arrow = self.ax.quiver([], [], [], [], color='r', scale=5)
        
        # 軌跡のプロット
        self.trail, = self.ax.plot([], [], 'r-', alpha=0.3, label='Movement Trail')

        self.ax.legend()
        self.ax.set_title('Real-time Position Tracking')
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        # plt.show(block=False)
        # テキスト表示用の軸の設定
        self.text_ax.axis('off')
        self.status_text = self.text_ax.text(0.05, 0.5, '', fontsize=10, 
                                            family='monospace', va='center')
        
        plt.tight_layout()
        plt.show(block=False)

    def update_plot(self):
        current_time = time.time()
        if current_time - self.last_update < 0.5:  # 100ms以上経過していない場合はスキップ
            return
        print(f"position_history: {len(self.position_history)}")
        if len(self.position_history) > 0:
            self.scatter.set_offsets(self.position_history[-1])
            # 軌跡の更新
            positions = np.array(self.position_history)
            self.trail.set_data(positions[:, 0], positions[:, 1])

        # ステータステキストの更新
        status_text = (
            f"Moving Forward: {self.moving_forward}, Heading: {self.heading:.1f}°, Magnitude: {self.acc_magnitude:.2f}\n"
            f"Position: ({self.position_history[-1][0]:.2f}, {self.position_history[-1][1]:.2f})\n"
            f"Acc: ({self.last_acc_x:.2f}, {self.last_acc_y:.2f}), "
            f"Gyro: ({self.last_gyro_x:.2f}, {self.last_gyro_y:.2f})"
        )
        self.status_text.set_text(status_text)
        
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
                
        self.last_update = current_time

    async def websocket_handler(self):
        print("WebSocketサーバー起動準備中...")
        try:
            async with websockets.serve(
                self.ws_server,
                "0.0.0.0",
                8080,
                ping_interval=None,
                ping_timeout=None
            ) as server:
                print("WebSocketサーバー起動完了: ws://localhost:8080")
                await asyncio.Future()
        except Exception as e:
            print(f"サーバーエラー: {e}")
    
    async def ws_server(self, websocket):
        client_address = websocket.remote_address
        print(f"新しいクライアント接続: {client_address}")
        await websocket.send(json.dumps({"status": "connected"}))
        async for message in websocket:
            try:
                data = json.loads(message)
                self.last_acc_x = data['acc_x'] if abs(data['acc_x']) > self.acc_data_threshold else 0.0
                self.last_acc_y = data['acc_y'] if abs(data['acc_y']) > self.acc_data_threshold else 0.0
                self.last_acc_z = data['acc_z'] if abs(data['acc_z']) > self.acc_data_threshold else 0.0
                self.last_gyro_x = data['gyro_x'] if abs(data['gyro_x']) > self.gyro_data_threshold else 0.0
                self.last_gyro_y = data['gyro_y'] if abs(data['gyro_y']) > self.gyro_data_threshold else 0.0
                self.last_gyro_z = data['gyro_z'] if abs(data['gyro_z']) > self.gyro_data_threshold else 0.0
                self.last_roll = data['roll']
                self.last_pitch = data['pitch']
                self.last_yaw = data['yaw']
                # IMUデータの処理と姿勢推定
                acc_data = np.array([self.last_acc_x, self.last_acc_y, self.last_acc_z - 1.0])  # スケール調整
                gyro_data = np.array([self.last_gyro_x, self.last_gyro_y, self.last_gyro_z])

                # position = np.array([self.last_position[0], self.last_position[1]])
                self.move_imu_data(self.last_acc_x, self.last_acc_y, self.last_yaw)
                # self.position_history.append(self.last_position) 

                # if self.moving_forward:
                    # self.position_history.append(position)
                self.update_plot()
                
                # デバッグ出力
                # print(f"position: {self.last_position}")
                # print(f"Moving Forward: {self.moving_forward}, Heading: {self.heading:.1f}°, Magnitude: {self.acc_magnitude:.2f}")
                # print(f"Roll: {self.last_roll:.1f}°, Pitch: {self.last_pitch:.1f}°, Yaw: {self.last_yaw:.1f}°")
                # print(f"acc_x: {acc_data[0]}, acc_y: {acc_data[1]}, acc_z: {acc_data[2]}, gyro_x: {gyro_data[0]}, gyro_y: {gyro_data[1]}, gyro_z: {gyro_data[2]}")
                await websocket.send(json.dumps({"status": "received"}))
            except Exception as e:
                print(f"Error processing message: {e}")

async def main():
    tracker = PositionTracker()
    await tracker.websocket_handler()

if __name__ == "__main__":
    plt.ion()  # インタラクティブモードを有効化
    asyncio.run(main())