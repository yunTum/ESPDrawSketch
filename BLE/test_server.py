import asyncio
import websockets
import json
import datetime
import logging
import socket
import csv
import os

# ログ設定
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# 自身のIPアドレスを取得
def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

async def websocket_server(websocket):
    client_address = websocket.remote_address
    logging.info(f"新しいクライアント接続: {client_address}")

    # CSVファイル名を現在の日時で生成
    csv_filename = f"./log/sensor_log_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    # CSVヘッダーを書き込み
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Timestamp', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'RSSI1', 'RSSI2'])
    
    
    try:
        # 接続確立時のメッセージ送信
        await websocket.send(json.dumps({"status": "connected"}))
        logging.info("接続確認メッセージを送信しました")
        
        async for message in websocket:
            logging.debug(f"受信データ (raw): {message}")
            try:
                data = json.loads(message)
                logging.info(f"処理済みデータ: {data}")
                
                # データの表示
                print(f"\n[{datetime.datetime.now()}]")
                print(f"加速度: X={data.get('acc_x',0)/100:.3f}, Y={data.get('acc_y',0)/100:.3f}, Z={data.get('acc_z',0)/100:.3f}")
                print(f"角速度: X={data.get('gyro_x',0)/10:.3f}, Y={data.get('gyro_y',0)/10:.3f}, Z={data.get('gyro_z',0)/10:.3f}")
                print(f"RSSI: Beacon1={data.get('rssi1', 'N/A')}, Beacon2={data.get('rssi2', 'N/A')}")
                timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
                # CSVにデータを追記
                with open(csv_filename, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        timestamp,
                        data.get('acc_x', 0)/100,
                        data.get('acc_y', 0)/100,
                        data.get('acc_z', 0)/100,
                        data.get('gyro_x', 0)/10,
                        data.get('gyro_y', 0)/10,
                        data.get('gyro_z', 0)/10,
                        data.get('rssi1', 'N/A'),
                        data.get('rssi2', 'N/A')
                    ])
                # 応答を送信
                await websocket.send(json.dumps({"status": "received"}))
                
            except json.JSONDecodeError as e:
                logging.error(f"JSON解析エラー: {e}")
                logging.error(f"受信データ: {message}")
            except Exception as e:
                logging.error(f"データ処理エラー: {e}", exc_info=True)
                
    except websockets.exceptions.ConnectionClosed as e:
        logging.info(f"クライアント切断 ({client_address}): {e}")
    except Exception as e:
        logging.error(f"予期せぬエラー: {e}", exc_info=True)

async def main():
    ip = get_ip()
    port = 8080
    
    logging.info(f"サーバー起動準備中... (IP: {ip}, Port: {port})")
    
    try:
        async with websockets.serve(
            websocket_server,
            "0.0.0.0",
            port,
            ping_interval=None,
            ping_timeout=None,
            close_timeout=None
        ) as server:
            logging.info(f"WebSocketサーバー起動: ws://{ip}:{port}")
            await asyncio.Future()  # サーバーを永続的に実行
            
    except Exception as e:
        logging.error(f"サーバー起動エラー: {e}", exc_info=True)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("サーバーを停止します")
    except Exception as e:
        logging.error(f"メインプロセスエラー: {e}", exc_info=True)