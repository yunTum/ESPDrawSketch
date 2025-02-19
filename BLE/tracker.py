import asyncio
import websockets
import json
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
from Common.MadgwickAHRS import MadgwickAHRS

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
        # ビーコンの位置（x, y）メートル単位
        self.beacon_positions = {
            'BEACON-1': np.array([0.0, 0.0, 0.5]),
            'BEACON-2': np.array([3.2, 0.0, 0.5]),
            'BEACON-3': np.array([0.0, 2.8, 0.5])
        }
        # self.kalman_filter = KalmanFilter()
        self.last_position = np.array([0.0, 0.0, 0.0])  # 初期位置
        self.last_velocity = np.array([0.0, 0.0, 0.0])  # 初期速度
        self.dt = 0.1  # サンプリング時間（秒）
        # デックを使用してデータ履歴を管理（最大50点）
        self.position_history = deque(maxlen=50)
        # 移動平均用のバッファ
        self.rssi_buffers = {
            'BEACON-1': deque(maxlen=50),
            'BEACON-2': deque(maxlen=50),
            'BEACON-3': deque(maxlen=50)
        }
        self.last_update = time.time()
        self.setup_plot()
        self.last_valid_position = None
        self.position_threshold = 0.2
        # センサーデータの初期値を設定
        self.last_rssi1 = 0
        self.last_rssi2 = 0
        self.last_rssi3 = 0
        self.last_acc_x = 0
        self.last_acc_y = 0
        self.last_acc_z = 0
        self.last_gyro_x = 0
        self.last_gyro_y = 0
        self.last_gyro_z = 0
        self.moving_forward = False
        self.heading = 0  # 進行方向（度）
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

        self.acc_data_threshold = 0.5
        self.gyro_data_threshold = 1.0

        self.imu_error_threshold = 0.5  # ビーコンとIMUの位置の差の閾値（メートル）
        self.alpha = 0.7  # ビーコン位置の重み（0-1）

    def filter_rssi(self, rssi, beacon_id):
        # RSSIの異常値フィルタリング
        self.rssi_buffers[beacon_id].append(rssi)
        
        if len(self.rssi_buffers[beacon_id]) < 3:
            return rssi
        
        # 中央値フィルタを適用
        filtered_rssi = np.median(self.rssi_buffers[beacon_id])
        
        # 急激な変化を制限
        if len(self.rssi_buffers[beacon_id]) >= 2:
            last_rssi = list(self.rssi_buffers[beacon_id])[-2]
            max_change = 2  # 許容するRSSIの最大変化量
            if abs(filtered_rssi - last_rssi) > max_change:
                filtered_rssi = last_rssi + np.sign(filtered_rssi - last_rssi) * max_change
        
        return filtered_rssi
    
    # def process_imu_data(self, acc_data, gyro_data):
    #     # MadgwickAHRSフィルタを使用して姿勢を更新
    #     self.madgwick.update(
    #         gyro_data[0], gyro_data[1], gyro_data[2],
    #         acc_data[0], acc_data[1], acc_data[2]
    #     )
    #     # 加速度ベクトルから移動方向を計算
    #     movement_direction = np.arctan2(acc_data[1], acc_data[0])
    #     # 現在の姿勢（heading）と移動方向の差を計算
    #     direction_diff = abs(self.normalize_angle(movement_direction - self.heading))

    #     # 前進判定
    #     # 1. 加速度の大きさが閾値を超えている
    #     acc_magnitude = np.sqrt(acc_data[0]**2 + acc_data[1]**2)
    #     acc_threshold = 2.0

    #     # 2. 移動方向が前方向から一定角度以内
    #     is_forward_direction = direction_diff < self.movement_direction_threshold

    #     self.moving_forward = acc_magnitude > acc_threshold and is_forward_direction
        
    #     # 姿勢の取得
    #     self.heading = self.madgwick.getYaw()
        
    #     return acc_data[0], acc_data[1]

    def estimate_position_from_imu(self, acc_data, roll, pitch, yaw):
        """IMUデータから位置を推定"""
        # 姿勢角をラジアンに変換
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        
        # 回転行列を作成（ボディフレームからグローバルフレームへの変換）
        R = np.array([
            [np.cos(yaw_rad)*np.cos(pitch_rad), 
             np.cos(yaw_rad)*np.sin(pitch_rad)*np.sin(roll_rad) - np.sin(yaw_rad)*np.cos(roll_rad),
             np.cos(yaw_rad)*np.sin(pitch_rad)*np.cos(roll_rad) + np.sin(yaw_rad)*np.sin(roll_rad)],
            [np.sin(yaw_rad)*np.cos(pitch_rad),
             np.sin(yaw_rad)*np.sin(pitch_rad)*np.sin(roll_rad) + np.cos(yaw_rad)*np.cos(roll_rad),
             np.sin(yaw_rad)*np.sin(pitch_rad)*np.cos(roll_rad) - np.cos(yaw_rad)*np.sin(roll_rad)],
            [-np.sin(pitch_rad),
             np.cos(pitch_rad)*np.sin(roll_rad),
             np.cos(pitch_rad)*np.cos(roll_rad)]
        ])
        
        # 加速度をグローバル座標系に変換
        acc_global = R @ acc_data
        
        # 重力加速度を除去（Z軸）
        # acc_global[2] -= 1.0
        
        # 速度を更新（積分）
        velocity = self.last_velocity + acc_global[:2] * self.dt
        
        # 位置を更新（積分）
        new_position = self.last_position + velocity * self.dt

        # # ビーコン領域内に制限
        # if self.is_position_valid(new_position):
        #     self.last_position = new_position
        #     self.last_velocity = velocity
        # else:
        #     # 領域外の場合は速度をゼロにリセット
        #     self.last_velocity = np.array([0.0, 0.0, 0.0])
            
        return new_position
    
    def is_position_valid(self, position):
        """位置がビーコンで形成される領域内にあるかチェック"""
        # 各ビーコンからの距離を計算
        distances = []
        for beacon_pos in self.beacon_positions.values():
            d = np.linalg.norm(position - beacon_pos)
            distances.append(d)
        
        # ビーコンの最大通信範囲
        max_range = 4.0  # メートル
        
        # 基本的な境界チェック
        x_valid = 0 <= position[0] <= 3.2
        y_valid = 0 <= position[1] <= 2.8
        
        # 全てのビーコンの通信範囲内にあるか
        in_range = all(d <= max_range for d in distances)
        
        # 三角形の内部にあるかチェック（オプション）
        p1 = self.beacon_positions['BEACON-1']
        p2 = self.beacon_positions['BEACON-2']
        p3 = self.beacon_positions['BEACON-3']
        
        def area(x1, y1, x2, y2, x3, y3):
            return abs((x1*(y2-y3) + x2*(y3-y1)+ x3*(y1-y2))/2.0)
        
        A = area(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1])
        A1 = area(position[0], position[1], p2[0], p2[1], p3[0], p3[1])
        A2 = area(p1[0], p1[1], position[0], position[1], p3[0], p3[1])
        A3 = area(p1[0], p1[1], p2[0], p2[1], position[0], position[1])
        
        # 点が三角形の内部にある場合、3つの小三角形の面積の和が元の三角形の面積と等しくなる
        in_triangle = abs(A - (A1 + A2 + A3)) < 1e-10
        
        return x_valid and y_valid and in_range and in_triangle

    def process_imu_data(self, roll, pitch, yaw, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        # 移動平均
        self.acc_x_buffer.append(acc_x)
        self.acc_y_buffer.append(acc_y)
        self.acc_z_buffer.append(acc_z)
        self.gyro_x_buffer.append(gyro_x)
        self.gyro_y_buffer.append(gyro_y)
        self.gyro_z_buffer.append(gyro_z)
        self.roll_buffer.append(roll)
        self.pitch_buffer.append(pitch)
        self.yaw_buffer.append(yaw)
        avg_acc_x = np.mean(self.acc_x_buffer) if self.acc_x_buffer else acc_x
        avg_acc_y = np.mean(self.acc_y_buffer) if self.acc_y_buffer else acc_y
        avg_acc_z = np.mean(self.acc_z_buffer) if self.acc_z_buffer else acc_z
        avg_gyro_x = np.mean(self.gyro_x_buffer) if self.gyro_x_buffer else gyro_x
        avg_gyro_y = np.mean(self.gyro_y_buffer) if self.gyro_y_buffer else gyro_y
        avg_gyro_z = np.mean(self.gyro_z_buffer) if self.gyro_z_buffer else gyro_z
        avg_roll = np.mean(self.roll_buffer) if self.roll_buffer else roll
        avg_pitch = np.mean(self.pitch_buffer) if self.pitch_buffer else pitch
        avg_yaw = np.mean(self.yaw_buffer) if self.yaw_buffer else yaw
        roll, pitch, yaw = avg_roll, avg_pitch, avg_yaw
        acc_x, acc_y, acc_z = avg_acc_x, avg_acc_y, avg_acc_z
        gyro_x, gyro_y, gyro_z = avg_gyro_x, avg_gyro_y, avg_gyro_z

        # 姿勢の取得
        self.heading = yaw
        # 移動方向の計算
        movement_direction = np.arctan2(acc_y, acc_x) # ラジアン
        # 前進判定
        acc_magnitude = np.sqrt(acc_x**2 + acc_y**2)
        print(f"acc_magnitude: {acc_magnitude}")
        acc_threshold = 0.3
        is_forward_direction = abs(np.degrees(movement_direction) - self.heading) < self.movement_direction_threshold
        self.moving_forward = acc_magnitude > acc_threshold and is_forward_direction
        return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw

    def normalize_angle(self, angle):
        """角度を-πからπの範囲に正規化"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def rssi_to_distance(self, rssi, beacon_id):
        # RSSIから距離を推定
        # 2.2m で-93dBm
        # 1.0m で-76dBm
        if rssi == 0:  # RSSI値が無効な場合
            return float('inf')
        # RSSI_1M:1mでのRSSI値
        # N:パスロス指数（環境に応じて2.0～4.0の間で調整）
        RSSI_params = {
            'BEACON-1': {'RSSI_1M': -82, 'N': 1.5},
            'BEACON-2': {'RSSI_1M': -79, 'N': 2.0},
            'BEACON-3': {'RSSI_1M': -77, 'N': 2.0}
            # 'BEACON-1': {'RSSI_1M': -82, 'N': 2.0},
            # 'BEACON-2': {'RSSI_1M': -78, 'N': 2.0},
            # 'BEACON-3': {'RSSI_1M': -68, 'N': 2.0}
        }
        params = RSSI_params[beacon_id]
        diagonal_distance = 10 ** ((params['RSSI_1M'] - rssi) / (10 * params['N']))

        return diagonal_distance
    
    def kalman_position(self, rssi1, rssi2, acc_data, gyro_data):
        # RSSI値の移動平均
        self.rssi1_buffer.append(rssi1)
        self.rssi2_buffer.append(rssi2)
        avg_rssi1 = np.mean(self.rssi1_buffer) if self.rssi1_buffer else rssi1
        avg_rssi2 = np.mean(self.rssi2_buffer) if self.rssi2_buffer else rssi2
        
        # BLEビーコンからの位置推定
        beacon_position = self.estimate_position(avg_rssi1, avg_rssi2)
        
        # IMUデータの処理
        acc_x, acc_y = self.process_imu_data(acc_data, gyro_data)
        
        # カルマンフィルタの予測ステップ
        self.kalman_filter.predict(acc_x, acc_y)
        
        # ビーコンによる測定値が有効な場合、更新ステップを実行
        if not np.any(np.isnan(beacon_position)):
            self.kalman_filter.update(beacon_position)

        # 速度の制限（急激な変化を防ぐ）
        max_velocity = 3.0  # m/s
        self.kalman_filter.state[2:4] = np.clip(self.kalman_filter.state[2:4], -max_velocity, max_velocity)
        
        # フィルタリングされた位置を返す
        return self.kalman_filter.state[:2]
    
    def estimate_position(self, rssi1, rssi2, rssi3):
        # if not self.moving_forward:
        #     # 前進していない場合は、最後の有効な位置を返す
        #     return self.last_valid_position if self.last_valid_position is not None else np.array([np.nan, np.nan])
        
        # RSSIのフィルタリング
        filtered_rssi1 = self.filter_rssi(rssi1, 'BEACON-1')
        filtered_rssi2 = self.filter_rssi(rssi2, 'BEACON-2')
        filtered_rssi3 = self.filter_rssi(rssi3, 'BEACON-3')

        # RSSIから距離を計算
        d1 = self.rssi_to_distance(rssi1, 'BEACON-1')
        d2 = self.rssi_to_distance(rssi2, 'BEACON-2')
        d3 = self.rssi_to_distance(rssi3, 'BEACON-3')
        
        # ビーコンの位置を取得
        p1 = self.beacon_positions['BEACON-1']
        p2 = self.beacon_positions['BEACON-2']
        p3 = self.beacon_positions['BEACON-3']
        print(f"Distances - B1: {d1:.2f}m, B2: {d2:.2f}m, B3: {d3:.2f}m")

        try:
            # 3点測位の行列計算
            # 連立方程式を解く
            # (x - x1)^2 + (y - y1)^2 + (z - z1)^2 = d1^2
            # (x - x2)^2 + (y - y2)^2 + (z - z2)^2 = d2^2
            # (x - x3)^2 + (y - y3)^2 + (z - z3)^2 = d3^2
            
            # 行列A（係数行列）
            A = np.array([
                [2*(p2[0] - p1[0]), 2*(p2[1] - p1[1]), 2*(p2[2] - p1[2])],
                [2*(p3[0] - p1[0]), 2*(p3[1] - p1[1]), 2*(p3[2] - p1[2])]
            ])
            
            # 行列b（定数項）
            b = np.array([
                [d1*d1 - d2*d2 - p1[0]*p1[0] + p2[0]*p2[0] - p1[1]*p1[1] + p2[1]*p2[1] - p1[2]*p1[2] + p2[2]*p2[2]],
                [d1*d1 - d3*d3 - p1[0]*p1[0] + p3[0]*p3[0] - p1[1]*p1[1] + p3[1]*p3[1] - p1[2]*p1[2] + p3[2]*p3[2]]
            ])
            
            # 最小二乗法で解を求める
            position = np.linalg.solve(A.T @ A, A.T @ b)
            position = position.flatten()
            
            # 結果の妥当性チェック
            # 各ビーコンからの計算距離と測定距離の差を確認
            # errors = []
            # for p, d in [(p1, d1), (p2, d2), (p3, d3)]:
            #     calc_d = np.sqrt(np.sum((position - p)**2))
            #     error = abs(calc_d - d)
            #     errors.append(error)
            
            # print(f"Position errors: {[f'{e:.2f}m' for e in errors]}")
            
            # # エラーが大きすぎる場合は結果を棄却
            # if max(errors) > 2.0:  # 2mを閾値とする
            #     return np.array([np.nan, np.nan])
            
            # 結果を有効範囲内に制限
            position = np.array([
                min(max(position[0], 0.0), 3.2),  # x座標: 0.0 ~ 3.2m
                min(max(position[1], 0.0), 2.8),   # y座標: 0.0 ~ 2.8m
                min(max(position[2], 0.0), 3.0)   # z座標: 0.0 ~ 3.0m
            ])

            # # 前回の有効な位置がある場合、移動方向をチェック
            # if self.last_valid_position is not None and not np.any(np.isnan(position)):
            #     movement_vector = position - self.last_valid_position
            #     if np.any(movement_vector):  # ゼロベクトルでない場合
            #         movement_direction = np.arctan2(movement_vector[1], movement_vector[0]) # ラジアン
            #         direction_diff = abs(self.normalize_angle(np.degrees(movement_direction) - self.heading))
                    
            #         # 移動方向が前方向から大きくずれている場合、位置更新を却下
            #         if direction_diff > self.movement_direction_threshold:
            #             return self.last_valid_position
            
            # # # エラーチェック後、有効な位置を保存
            # if not np.any(np.isnan(position)):
            #     self.last_valid_position = position.copy()
            print(f"Position: {position}")
            return position
        
        except np.linalg.LinAlgError:
            print("行列計算エラー")
            return np.array([np.nan, np.nan])
        except Exception as e:
            print(f"位置推定エラー: {e}")
            return np.array([np.nan, np.nan])

    def correction_position(self, imu_position, beacon_position):
        if beacon_position is not None and not np.any(np.isnan(beacon_position)):
            if self.last_beacon_position is not None:
                # ビーコンの位置変化を計算
                beacon_velocity = (beacon_position - self.last_beacon_position) / self.dt
                # IMUとビーコンの位置の差を計算
                position_error = np.linalg.norm(imu_position[:2] - beacon_position[:2])
                if position_error > self.imu_error_threshold:
                    # 誤差が大きい場合、ビーコン位置を重視
                    corrected_position = np.array([
                        self.alpha * beacon_position[0] + (1 - self.alpha) * imu_position[0],
                        self.alpha * beacon_position[1] + (1 - self.alpha) * imu_position[1],
                        imu_position[2]  # Z軸はIMUの値を維持
                    ])
                    # 速度も補正
                    self.last_velocity[:2] = beacon_velocity
                else:
                    # 誤差が小さい場合、IMU位置を優先
                    corrected_position = imu_position
                
                self.last_beacon_position = beacon_position
            else:
                # 初回のビーコン位置
                corrected_position = np.array([
                    beacon_position[0],
                    beacon_position[1],
                    imu_position[2]
                ])
                self.last_beacon_position = beacon_position
        else:
            # ビーコン位置が無効な場合はIMUの位置を使用
            corrected_position = imu_position

        # 位置の有効性チェック
        if self.is_position_valid(corrected_position[:2]):
            self.last_position = corrected_position
            self.last_velocity = self.last_velocity
        else:
            # 領域外の場合は速度をゼロにリセット
            self.last_velocity = np.array([0.0, 0.0, 0.0])
        
        return self.last_position

    def setup_plot(self):
        plt.ion()  # インタラクティブモードを有効化
        # self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig, (self.ax, self.text_ax) = plt.subplots(2, 1, figsize=(8, 10), 
                                                    gridspec_kw={'height_ratios': [4, 1]})
        self.ax.set_xlim(-1, 4)
        self.ax.set_ylim(-1, 4)
        self.ax.grid(True)
        
        # 現在位置のプロット
        self.scatter = self.ax.scatter([], [], c='r', s=100, label='Current Position')

        # 向きを示す矢印
        self.direction_arrow = self.ax.quiver([], [], [], [], color='r', scale=5)
        
        # 軌跡のプロット
        self.trail, = self.ax.plot([], [], 'r-', alpha=0.3, label='Movement Trail')
        
        # ビーコンの位置をプロット
        # ビーコンで色を分ける
        for name, pos in self.beacon_positions.items():
            if name == 'BEACON-1':
                self.ax.plot(pos[0], pos[1], 'bs', label=name)
            else:
                self.ax.plot(pos[0], pos[1], 'rs', label=name)
        
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
            self.scatter.set_offsets(self.position_history)
        # if len(self.position_history) > 0:
        #     positions = np.array(list(self.position_history))
        #     # 異常値の除去
        #     valid_positions = positions[~np.isnan(positions).any(axis=1)]
            
        #     if len(valid_positions) > 0:
        #         current_pos = valid_positions[-1]
        #         # 現在位置の点をプロット
        #         self.scatter.set_offsets(current_pos)
                
                # # 向きを示す矢印を更新
                # arrow_length = 0.3  # 矢印の長さ
                # dx = arrow_length * np.cos(np.radians(self.heading))
                # dy = arrow_length * np.sin(np.radians(self.heading))
                
                # # 矢印を更新（現在位置から向きの方向へ）
                # self.direction_arrow.set_offsets(current_pos)
                # self.direction_arrow.set_UVC(dx, dy)
                
                # # 移動の軌跡を更新
                # if len(valid_positions) > 2:
                #     # 移動平均による平滑化
                #     window = 20
                #     smoothed = np.convolve(valid_positions[:, 0], np.ones(window)/window, mode='valid')
                #     valid_positions[window-1:, 0] = smoothed
                #     smoothed = np.convolve(valid_positions[:, 1], np.ones(window)/window, mode='valid')
                #     valid_positions[window-1:, 1] = smoothed
                
                # self.trail.set_data(valid_positions[:, 0], valid_positions[:, 1])
                
                # # 前進状態を色で表現（オプション）
                # if self.moving_forward:
                #     self.scatter.set_color('g')  # 前進中は緑
                #     self.direction_arrow.set_color('g')
                # else:
                #     self.scatter.set_color('r')  # 停止中は赤
                #     self.direction_arrow.set_color('r')
        # ステータステキストの更新
        status_text = (
            f"Moving Forward: {self.moving_forward}, Heading: {self.heading:.1f}°\n"
            f"Position: ({self.position_history[-1][0]:.2f}, {self.position_history[-1][1]:.2f}, {self.position_history[-1][2]:.2f})\n"
            f"RSSI: B1={self.last_rssi1:.1f}, B2={self.last_rssi2:.1f}, B3={self.last_rssi3:.1f}\n"
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
                # センサーデータの保存（プロット用）
                self.last_rssi1 = data['rssi1']
                self.last_rssi2 = data['rssi2']
                self.last_rssi3 = data['rssi3']
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
                acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw = self.process_imu_data(self.last_roll, self.last_pitch, self.last_yaw, self.last_acc_x, self.last_acc_y, self.last_acc_z, self.last_gyro_x, self.last_gyro_y, self.last_gyro_z)
                acc_data = np.array([acc_x, acc_y, acc_z - 1.0])  # スケール調整
                gyro_data = np.array([gyro_x, gyro_y, gyro_z])
                # 50回ごとに姿勢を表示
                self.get_imu_count += 1
                if self.get_imu_count % 50 == 0:
                    # IMUによる位置推定
                    imu_position = self.estimate_position_from_imu(
                        acc_data,
                        roll,
                        pitch,
                        yaw
                    )
                    beacon_position = self.estimate_position(
                        data['rssi1'],
                        data['rssi2'],
                        data['rssi3']
                    )
                    position = self.correction_position(imu_position, beacon_position)
                    # position = self.kalman_position(data['rssi1'], data['rssi2'], acc_data, gyro_data)
                    # if not np.any(np.isnan(position)):
                    self.position_history.append(position) 

                    # if self.moving_forward:
                        # self.position_history.append(position)
                    self.update_plot()
                    
                    # デバッグ出力
                    print(f"position: {position}")
                    print(f"Moving Forward: {self.moving_forward}, Heading: {self.heading:.1f}°")
                    print(f"Roll: {self.last_roll:.1f}°, Pitch: {self.last_pitch:.1f}°, Yaw: {self.last_yaw:.1f}°")
                    print(f"RSSI1: {self.last_rssi1}, RSSI2: {self.last_rssi2}, RSSI3: {self.last_rssi3}")
                    print(f"acc_x: {acc_data[0]}, acc_y: {acc_data[1]}, acc_z: {acc_data[2]}, gyro_x: {gyro_data[0]}, gyro_y: {gyro_data[1]}, gyro_z: {gyro_data[2]}")
                await websocket.send(json.dumps({"status": "received"}))
            except Exception as e:
                print(f"Error processing message: {e}")

async def main():
    tracker = PositionTracker()
    await tracker.websocket_handler()

if __name__ == "__main__":
    plt.ion()  # インタラクティブモードを有効化
    asyncio.run(main())