import asyncio
import websockets
import json
import random

async def send_bme_data():
    uri = "ws://localhost:8000/ws/robot/test_robot"
    async with websockets.connect(uri) as websocket:
        print("Connected to WebSocket server as 'test_robot'")
        while True:
            data = {
                "type": "sensor_data",
                "data": {
                    "bme280": {
                        "temperature_celsius": round(random.uniform(20.0, 30.0), 2),
                        "humidity_percent": round(random.uniform(40.0, 60.0), 2),
                        "pressure_hpa": round(random.uniform(1000.0, 1020.0), 2)
                    }
                }
            }
            await websocket.send(json.dumps(data))
            print(f"Sent: {data}")
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(send_bme_data())
