import asyncio
import websockets
import json

async def test_connection():
    uri = "ws://localhost:8080"
    async with websockets.connect(uri) as websocket:
        print("接続成功")
        # テストメッセージの送信
        test_data = {
            "ax": 100,
            "ay": 200,
            "az": 300,
            "gx": 10,
            "gy": 20,
            "gz": 30,
            "r1": -70,
            "r2": -80,
            "t": 1000
        }
        await websocket.send(json.dumps(test_data))
        print("テストデータ送信完了")

asyncio.run(test_connection())