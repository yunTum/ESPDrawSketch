import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from tkinter import filedialog
import tkinter as tk
import os

def analyze_acceleration_data(csv_file):
    acc_threshold = 0.03  # 加速度の閾値 (m/s²)
    vel_threshold = 0.05  # 速度の閾値 (m/s)
    damping = 0.85       # 速度の減衰係数
    # CSVファイルの読み込み
    df = pd.read_csv(csv_file)
    
    # タイムスタンプを datetime 型に変換
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    
    # 経過時間（秒）の計算
    df['time'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
    
    # 移動平均でノイズを軽減（必要に応じて調整）
    window_size = 5
    df['acc_x_smooth'] = df['acc_x'].rolling(window=window_size, center=True).mean()
    df['acc_y_smooth'] = df['acc_y'].rolling(window=window_size, center=True).mean()
    df['acc_z_smooth'] = df['acc_z'].rolling(window=window_size, center=True).mean()
    
    # NaNを0で埋める
    df = df.fillna(0)
    
    # 速度の計算（加速度の積分）
    vel_x = cumulative_trapezoid(df['acc_x_smooth'], df['time'], initial=0) * damping
    vel_y = cumulative_trapezoid(df['acc_y_smooth'], df['time'], initial=0) * damping
    vel_z = cumulative_trapezoid(df['acc_z_smooth'], df['time'], initial=0) * damping
    
    # 位置の計算（速度の積分）
    pos_x = cumulative_trapezoid(vel_x, df['time'], initial=0)
    pos_y = cumulative_trapezoid(vel_y, df['time'], initial=0)
    pos_z = cumulative_trapezoid(vel_z, df['time'], initial=0)
    
    # グラフの描画
    fig = plt.figure(figsize=(15, 10))
    
    # 加速度のグラフ
    ax1 = fig.add_subplot(311)
    ax1.plot(df['time'], df['acc_x_smooth'], label='X')
    ax1.plot(df['time'], df['acc_y_smooth'], label='Y')
    ax1.plot(df['time'], df['acc_z_smooth'], label='Z')
    ax1.set_title('Acceleration')
    ax1.set_xlabel('Time (sec)')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.grid(True)
    ax1.legend()
    
    # 速度のグラフ
    ax2 = fig.add_subplot(312)
    ax2.plot(df['time'], vel_x, label='X')
    ax2.plot(df['time'], vel_y, label='Y')
    ax2.plot(df['time'], vel_z, label='Z')
    ax2.set_title('Velocity')
    ax2.set_xlabel('Time (sec)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True)
    ax2.legend()
    
    # 位置のグラフ
    ax3 = fig.add_subplot(313)
    ax3.plot(df['time'], pos_x, label='X')
    ax3.plot(df['time'], pos_y, label='Y')
    ax3.plot(df['time'], pos_z, label='Z')
    ax3.set_title('Position')
    ax3.set_xlabel('Time (sec)')
    ax3.set_ylabel('Position (m)')
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    plt.show()
    
    # 3D軌跡の表示
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pos_x, pos_y, pos_z)
    ax.set_title('3D-Trajectory')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.show()
    
    # 総移動距離の計算
    dx = np.diff(pos_x)
    dy = np.diff(pos_y)
    dz = np.diff(pos_z)
    total_distance = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))
    print(f"総移動距離: {total_distance:.2f} m")

def create_3d_kalman_filter(dt):
    kf = KalmanFilter(dim_x=9, dim_z=3)  # 状態: [x, y, z, vx, vy, vz, ax, ay, az]
    
    # 状態遷移行列
    kf.F = np.array([
        [1, 0, 0, dt, 0, 0, dt**2/2, 0, 0],
        [0, 1, 0, 0, dt, 0, 0, dt**2/2, 0],
        [0, 0, 1, 0, 0, dt, 0, 0, dt**2/2],
        [0, 0, 0, 1, 0, 0, dt, 0, 0],
        [0, 0, 0, 0, 1, 0, 0, dt, 0],
        [0, 0, 0, 0, 0, 1, 0, 0, dt],
        [0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1]
    ])
    
    # 観測行列（加速度のみ観測）
    kf.H = np.array([
        [0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1]
    ])
    
    # 初期状態の不確かさ
    kf.P *= 1000
    
    # プロセスノイズ
    kf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=0.1, block_size=3)
    
    # 測定ノイズ
    kf.R = np.eye(3) * 0.1
    
    return kf

def create_9dof_kalman_filter(dt):
    """9自由度（加速度3軸、ジャイロ3軸、姿勢3軸）のカルマンフィルタを作成"""
    kf = KalmanFilter(dim_x=12, dim_z=6)  # 状態: [x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw]
    
    # 状態遷移行列
    kf.F = np.eye(12)
    kf.F[0:3, 3:6] = np.eye(3) * dt
    kf.F[3:6, 6:9] = np.eye(3) * dt
    
    # 観測行列（加速度とジャイロを観測）
    kf.H = np.zeros((6, 12))
    kf.H[0:3, 6:9] = np.eye(3)  # 加速度
    kf.H[3:6, 9:12] = np.eye(3)  # 角速度
    
    # 初期状態の不確かさ
    kf.P = np.eye(12) * 100
    kf.P[0:3, 0:3] *= 0.1    # 位置の初期不確かさ
    kf.P[3:6, 3:6] *= 0.1    # 速度の初期不確かさ
    kf.P[6:9, 6:9] *= 1.0   # 加速度の初期不確かさ
    kf.P[9:12, 9:12] *= 0.1  # 姿勢の初期不確かさ

    # プロセスノイズ
    kf.Q = np.eye(12)
    kf.Q[0:3, 0:3] *= 0.01   # 位置（m）
    kf.Q[3:6, 3:6] *= 0.1    # 速度（m/s）
    kf.Q[6:9, 6:9] *= 1.0    # 加速度（m/s²）
    kf.Q[9:12, 9:12] *= 0.1  # 姿勢（rad）
    
    # 測定ノイズ
    kf.R = np.eye(6)
    kf.R[0:3, 0:3] *= 2.0    # 加速度ノイズ（m/s²）
    kf.R[3:6, 3:6] *= 1.0    # 角速度ノイズ（rad/s）
    
    return kf

def analyze_acceleration_data_kalman(csv_file):
    """カルマンフィルタを使用して加速度データから位置を推定する"""
    # CSVファイルの読み込み
    df = pd.read_csv(csv_file)
    
    # タイムスタンプを datetime 型に変換
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    
    # 経過時間（秒）の計算
    df['time'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
    
    # 移動平均でノイズを軽減
    window_size = 5
    df['acc_x_smooth'] = df['acc_x'].rolling(window=window_size, center=True).mean()
    df['acc_y_smooth'] = df['acc_y'].rolling(window=window_size, center=True).mean()
    df['acc_z_smooth'] = df['acc_z'].rolling(window=window_size, center=True).mean()
    
    # NaNを0で埋める
    df = df.fillna(0)
    
    # サンプリング間隔の計算
    dt = np.mean(np.diff(df['time']))
    
    # カルマンフィルタの初期化
    kf = create_3d_kalman_filter(dt)
    kf.x = np.zeros(9)  # 初期状態
    
    # カルマンフィルタによる位置推定
    positions = np.zeros((len(df), 3))
    velocities = np.zeros((len(df), 3))
    accelerations = np.zeros((len(df), 3))
    
    for i in range(len(df)):
        z = np.array([df['acc_x_smooth'].iloc[i],
                     df['acc_y_smooth'].iloc[i],
                     df['acc_z_smooth'].iloc[i]])
        kf.predict()
        kf.update(z)
        positions[i] = kf.x[:3]
        velocities[i] = kf.x[3:6]
        accelerations[i] = kf.x[6:]
    
    # グラフの描画
    fig = plt.figure(figsize=(15, 10))
    
    # 加速度のグラフ
    ax1 = fig.add_subplot(311)
    ax1.plot(df['time'], accelerations[:, 0], label='X (Kalman)')
    ax1.plot(df['time'], accelerations[:, 1], label='Y (Kalman)')
    ax1.plot(df['time'], accelerations[:, 2], label='Z (Kalman)')
    ax1.set_title('Acceleration (Kalman Filter)')
    ax1.set_xlabel('Time (sec)')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.grid(True)
    ax1.legend()
    
    # 速度のグラフ
    ax2 = fig.add_subplot(312)
    ax2.plot(df['time'], velocities[:, 0], label='X (Kalman)')
    ax2.plot(df['time'], velocities[:, 1], label='Y (Kalman)')
    ax2.plot(df['time'], velocities[:, 2], label='Z (Kalman)')
    ax2.set_title('Velocity (Kalman Filter)')
    ax2.set_xlabel('Time (sec)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True)
    ax2.legend()
    
    # 位置のグラフ
    ax3 = fig.add_subplot(313)
    ax3.plot(df['time'], positions[:, 0], label='X (Kalman)')
    ax3.plot(df['time'], positions[:, 1], label='Y (Kalman)')
    ax3.plot(df['time'], positions[:, 2], label='Z (Kalman)')
    ax3.set_title('Position (Kalman Filter)')
    ax3.set_xlabel('Time (sec)')
    ax3.set_ylabel('Position (m)')
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    plt.show()
    
    # 3D軌跡の表示
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2])
    ax.set_title('3D-Trajectory (Kalman Filter)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.show()
    
    # 総移動距離の計算
    dx = np.diff(positions[:, 0])
    dy = np.diff(positions[:, 1])
    dz = np.diff(positions[:, 2])
    total_distance = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))
    print(f"総移動距離（カルマンフィルタ）: {total_distance:.2f} m")

def analyze_imu_data_kalman(csv_file):
    """カルマンフィルタを使用して IMU データから位置と姿勢を推定する"""
    # CSVファイルの読み込み
    df = pd.read_csv(csv_file)
    
    # タイムスタンプを datetime 型に変換
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    df['time'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
    
    # 静止状態の検出
    static_threshold = 0.05
    acc_magnitude = np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2)
    is_static = abs(acc_magnitude - 0.2) < static_threshold 

    # 単位変換：g →
    for axis in ['x', 'y', 'z']:
        df[f'acc_{axis}'] = df[f'acc_{axis}']
    
    # 単位変換：度/秒 → ラジアン/秒
    DEG_TO_RAD = np.pi / 180.0
    for axis in ['x', 'y', 'z']:
        df[f'gyro_{axis}'] = df[f'gyro_{axis}'] * DEG_TO_RAD

    # 移動平均でノイズを軽減
    window_size = 3
    for axis in ['x', 'y', 'z']:
        df[f'acc_{axis}_smooth'] = df[f'acc_{axis}'].rolling(window=window_size, center=True).mean()
        df[f'gyro_{axis}_smooth'] = df[f'gyro_{axis}'].rolling(window=window_size, center=True).mean()
    
    # NaNを0で埋める
    df = df.fillna(0)
    
    # サンプリング間隔の計算
    dt = np.mean(np.diff(df['time']))
    print(f"平均サンプリング間隔: {dt*1000:.1f}ms")
    print(f"サンプリング周波数: {1/dt:.1f}Hz")
    print(f"計測時間: {df['time'].max():.1f}秒")
    print(f"データポイント数: {len(df)}")
    
    # カルマンフィルタの初期化
    kf = create_9dof_kalman_filter(dt)
    kf.x = np.zeros(12)
    
    # 結果を格納する配列
    positions = np.zeros((len(df), 3))
    velocities = np.zeros((len(df), 3))
    accelerations = np.zeros((len(df), 3))
    attitudes = np.zeros((len(df), 3))  # roll, pitch, yaw
    
    for i in range(len(df)):
        # 加速度とジャイロの測定値
        z = np.array([
            df['acc_x_smooth'].iloc[i],
            df['acc_y_smooth'].iloc[i],
            df['acc_z_smooth'].iloc[i],
            df['gyro_x_smooth'].iloc[i],
            df['gyro_y_smooth'].iloc[i],
            df['gyro_z_smooth'].iloc[i]
        ])
        # 静止状態での速度リセット
        if is_static.iloc[i]:
            kf.x[3:6] = 0
        
        kf.predict()
        kf.update(z)
        
        positions[i] = kf.x[0:3]
        velocities[i] = kf.x[3:6]
        accelerations[i] = kf.x[6:9]
        attitudes[i] = kf.x[9:12]
    
    # グラフの描画
    fig = plt.figure(figsize=(15, 12))
    
    # 加速度のグラフ
    ax1 = fig.add_subplot(411)
    ax1.plot(df['time'], accelerations[:, 0], label='X')
    ax1.plot(df['time'], accelerations[:, 1], label='Y')
    ax1.plot(df['time'], accelerations[:, 2], label='Z')
    ax1.set_title('Acceleration (Kalman Filter)')
    ax1.set_xlabel('Time (sec)')
    ax1.set_ylabel('Acceleration (m/s²)')
    ax1.grid(True)
    ax1.legend()
    
    # 速度のグラフ
    ax2 = fig.add_subplot(412)
    ax2.plot(df['time'], velocities[:, 0], label='X')
    ax2.plot(df['time'], velocities[:, 1], label='Y')
    ax2.plot(df['time'], velocities[:, 2], label='Z')
    ax2.set_title('Velocity (Kalman Filter)')
    ax2.set_xlabel('Time (sec)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True)
    ax2.legend()
    
    # 位置のグラフ
    ax3 = fig.add_subplot(413)
    ax3.plot(df['time'], positions[:, 0], label='X')
    ax3.plot(df['time'], positions[:, 1], label='Y')
    ax3.plot(df['time'], positions[:, 2], label='Z')
    ax3.set_title('Position (Kalman Filter)')
    ax3.set_xlabel('Time (sec)')
    ax3.set_ylabel('Position (m)')
    ax3.grid(True)
    ax3.legend()
    
    # 姿勢のグラフ
    ax4 = fig.add_subplot(414)
    ax4.plot(df['time'], np.rad2deg(attitudes[:, 0]), label='Roll')
    ax4.plot(df['time'], np.rad2deg(attitudes[:, 1]), label='Pitch')
    ax4.plot(df['time'], np.rad2deg(attitudes[:, 2]), label='Yaw')
    ax4.set_title('Attitude (Kalman Filter)')
    ax4.set_xlabel('Time (sec)')
    ax4.set_ylabel('Angle (deg)')
    ax4.grid(True)
    ax4.legend()
    
    plt.tight_layout()
    plt.show()
    
    # 3D軌跡の表示
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2])
    ax.set_title('3D-Trajectory (Kalman Filter)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    plt.show()
    
    # 総移動距離の計算
    dx = np.diff(positions[:, 0])
    dy = np.diff(positions[:, 1])
    dz = np.diff(positions[:, 2])
    total_distance = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))
    print(f"総移動距離（カルマンフィルタ）: {total_distance:.2f} m")


def select_csv_file():
    """CSVファイルを選択するダイアログを表示する"""
    root = tk.Tk()
    root.withdraw()  # メインウィンドウを非表示
    
    # 初期ディレクトリを現在のスクリプトのディレクトリに設定
    initial_dir = os.path.dirname(os.path.abspath(__file__))
    
    file_path = filedialog.askopenfilename(
        initialdir=initial_dir,
        title="加速度データのCSVファイルを選択",
        filetypes=[("CSVファイル", "*.csv"), ("すべてのファイル", "*.*")]
    )
    
    return file_path


if __name__ == "__main__":
    # csv_file = "acceleration_data_20250214_144523.csv"  # CSVファイル名を適宜変更
    # analyze_acceleration_data(csv_file)
    # ファイル選択ダイアログを表示
    csv_file = select_csv_file()
    
    if csv_file:  # ファイルが選択された場合
        print(f"選択されたファイル: {csv_file}")
        print("\n単純積分による解析:")
        analyze_acceleration_data(csv_file)
        print("\nカルマンフィルタによる解析:")
        analyze_imu_data_kalman(csv_file)
    else:
        print("ファイルが選択されませんでした。")