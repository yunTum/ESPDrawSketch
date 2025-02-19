import socket
import struct
import time
from datetime import datetime
class AccelerationReceiver:
    def __init__(self):
        self.UDP_IP = "0.0.0.0"
        self.UDP_PORT = 4215

        # UDPソケットの設定
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        print(f"UDPリスニング開始: {self.UDP_IP}:{self.UDP_PORT}")
        self.count = 0

        # CSVファイルの準備
        current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f"acceleration_data_{current_time}.csv"
        with open(self.csv_filename, 'w', newline='') as f:
            f.write("timestamp,sequence,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n")
        print(f"データを {self.csv_filename} に記録します")

    def decode_float_signed(self, data):
        # 符号付きfloat値のデコード
        sign = data[0]  # 符号ビット
        value = (data[1] << 8) | data[2]  # 値の部分
        return -value / 100.0 if sign else value / 100.0

    def receive_data(self):
        while self.count < 500:
            try:
                data, addr = self.sock.recvfrom(21)
                
                if len(data) != 21:
                    continue

                # パケットの解析
                header = data[0]
                if header != 0x55:
                    continue
                
                # データの取得
                seq_no = (data[1] << 8) | data[2]
                acc_x = self.decode_float_signed(data[3:6])
                acc_y = self.decode_float_signed(data[6:9])
                acc_z = self.decode_float_signed(data[9:12])
                gyro_x = self.decode_float_signed(data[12:15])
                gyro_y = self.decode_float_signed(data[15:18])
                gyro_z = self.decode_float_signed(data[18:21])
                # タイムスタンプの取得
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

                # CSVに記録
                with open(self.csv_filename, 'a', newline='') as f:
                    f.write(f"{timestamp},{seq_no},{acc_x:.2f},{acc_y:.2f},{acc_z:.2f},{gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f}\n")

                print(f"シーケンス番号: {seq_no}")
                print(f"加速度 X: {acc_x:.2f} Y: {acc_y:.2f} Z: {acc_z:.2f}")
                print(f"ジャイロ X: {gyro_x:.2f} Y: {gyro_y:.2f} Z: {gyro_z:.2f}")
                print("-" * 40)
                self.count += 1

            except Exception as e:
                print(f"エラー発生: {e}")

if __name__ == "__main__":
    receiver = AccelerationReceiver()
    receiver.receive_data()