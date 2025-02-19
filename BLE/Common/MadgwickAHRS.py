import numpy as np

class MadgwickAHRS:
    def __init__(self, beta=0.1, sample_freq=100.0):
        self.beta = beta
        self.sample_freq = sample_freq
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # クォータニオン初期値
        
    def update(self, gx, gy, gz, ax, ay, az):
        # ジャイロデータをラジアンに変換
        gx = gx * np.pi / 180.0
        gy = gy * np.pi / 180.0
        gz = gz * np.pi / 180.0
        
        # クォータニオンの値を取得
        q1, q2, q3, q4 = self.q
        
        # 加速度の正規化
        norm = np.sqrt(ax * ax + ay * ay + az * az)
        if norm != 0.0:
            ax, ay, az = ax/norm, ay/norm, az/norm
            
        # 勾配降下法
        F = np.array([
            2.0 * (q2 * q4 - q1 * q3) - ax,
            2.0 * (q1 * q2 + q3 * q4) - ay,
            2.0 * (0.5 - q2 * q2 - q3 * q3) - az
        ])
        
        J = np.array([
            [-2.0*q3,  2.0*q4, -2.0*q1,  2.0*q2],
            [ 2.0*q2,  2.0*q1,  2.0*q4,  2.0*q3],
            [    0.0, -4.0*q2, -4.0*q3,     0.0]
        ])
        
        step = J.T @ F
        step = step / np.linalg.norm(step)
        
        # クォータニオンの更新レート
        qDot = 0.5 * np.array([
            -q2*gx - q3*gy - q4*gz,
            q1*gx + q3*gz - q4*gy,
            q1*gy - q2*gz + q4*gx,
            q1*gz + q2*gy - q3*gx
        ]) - self.beta * step
        
        # クォータニオンの積分
        self.q += qDot / self.sample_freq
        # 正規化
        self.q = self.q / np.linalg.norm(self.q)
        
    def getYaw(self):
        """ヨー角（Z軸周りの回転）を取得"""
        q1, q2, q3, q4 = self.q
        return np.arctan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))
    
    def getPitch(self):
        """ピッチ角（Y軸周りの回転）を取得"""
        q1, q2, q3, q4 = self.q
        return np.arcsin(2.0 * (q1 * q3 - q4 * q2))
    
    def getRoll(self):
        """ロール角（X軸周りの回転）を取得"""
        q1, q2, q3, q4 = self.q
        return np.arctan2(2.0 * (q1 * q2 + q3 * q4), 1.0 - 2.0 * (q2 * q2 + q3 * q3))