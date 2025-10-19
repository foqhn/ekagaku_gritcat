#how to run FastAPI
# uvicorn main:app --reload
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image

import asyncio
import cv2
import threading
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from starlette.websockets import WebSocket, WebSocketDisconnect

from cv_bridge import CvBridge
import numpy as np

from src.oled import OLEDDisplay
from src.motor_controller import MotorDriver

# --- グローバル変数とロック ---
latest_imu_msg = None
latest_image_msg = None
imu_lock = threading.Lock()
image_lock = threading.Lock()

# --- ROS 2 ノード ---
class RosSubscriberNode(Node):
    def __init__(self):
        super().__init__('ros_fastapi_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu, '/bno055/imu', self.imu_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('ROS Subscriber Node has been started.')

    def imu_callback(self, msg):
        global latest_imu_msg
        with imu_lock:
            latest_imu_msg = msg

    def image_callback(self, msg):
        global latest_image_msg
        with image_lock:
            latest_image_msg = msg
        self.get_logger().info('<<<<< Image data received by ROS node! >>>>>')

def run_ros_node():
    rclpy.init()
    ros_node = RosSubscriberNode()
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

# --- FastAPI アプリケーション ---
app = FastAPI()

def imu_to_dict(imu_msg: Imu):
    if not imu_msg:
        return None
    return {
        'header': {
            'stamp': {'sec': imu_msg.header.stamp.sec, 'nanosec': imu_msg.header.stamp.nanosec},
            'frame_id': imu_msg.header.frame_id
        },
        'orientation': {
            'x': imu_msg.orientation.x, 'y': imu_msg.orientation.y,
            'z': imu_msg.orientation.z, 'w': imu_msg.orientation.w
        },
        'angular_velocity': {
            'x': imu_msg.angular_velocity.x, 'y': imu_msg.angular_velocity.y, 'z': imu_msg.angular_velocity.z
        },
        'linear_acceleration': {
            'x': imu_msg.linear_acceleration.x, 'y': imu_msg.linear_acceleration.y, 'z': imu_msg.linear_acceleration.z
        }
    }

@app.websocket("/sensors/imu")
async def websocket_imu_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with imu_lock:
                imu_data = imu_to_dict(latest_imu_msg)
            
            if imu_data:
                await websocket.send_json(imu_data)
            
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        print("IMU client disconnected")
    except Exception as e:
        print(f"An error occurred in IMU websocket: {e}")
    # finally:
    #     await websocket.close() # <-- この行を削除またはコメントアウト


@app.websocket("/sensors/image")
async def websocket_image_endpoint(websocket: WebSocket):
    # ★★★ 接続が確立されたかを確認するログを追加 ★★★
    try:
        await websocket.accept()
        print("!!! Image WebSocket client connected successfully !!!")
    except Exception as e:
        print(f"!!! WebSocket accept failed: {e} !!!")
        return

    bridge = CvBridge()
    try:
        while True:
            image_to_send = None
            cv_image = None # cv_imageをループの先頭で初期化

            with image_lock:
                if latest_image_msg:
                    # ★★★ 画像変換処理をtry-exceptで囲み、エラーを捕捉する ★★★
                    try:
                        cv_image = bridge.imgmsg_to_cv2(latest_image_msg, desired_encoding='bgr8')
                    except Exception as e:
                        print(f"!!! Error in cv_bridge.imgmsg_to_cv2: {e} !!!")
                        # エラーが発生した場合、このループの残りはスキップ
                        continue

            # 画像変換が成功した場合のみエンコード処理に進む
            if cv_image is not None:
                try:
                    ret, frame = cv2.imencode('.jpg', cv_image)
                    if ret:
                        image_to_send = frame.tobytes()
                    else:
                        print("!!! cv2.imencode failed !!!")
                except Exception as e:
                    print(f"!!! Error in cv2.imencode: {e} !!!")


            if image_to_send:
                print(f">>>>> Sending image data via WebSocket (size: {len(image_to_send)} bytes)")
                await websocket.send_bytes(image_to_send)
                
            await asyncio.sleep(0.033)
            
    except WebSocketDisconnect:
        print("Image client disconnected")
    except Exception as e:
        print(f"An unexpected error occurred in Image websocket: {e}")


# --- HTMLフロントエンド ---
html = """
<!DOCTYPE html>
<html>
    <head>
        <title>ROS 2 FastAPI Viewer</title>
    </head>
    <body>
        <h1>ROS 2 Real-time Data Viewer</h1>
        
        <h2>IMU Data (/bno055/imu)</h2>
        <pre id="imu-data">Connecting...</pre>
        
        <h2>Camera Image (/camera/image_raw)</h2>
        <img id="camera-image" width="640" height="480" alt="Camera stream will appear here">
        
        <script>
            // --- IMU WebSocket ---
            const imu_ws = new WebSocket(`ws://${window.location.host}/sensors/imu`);
            imu_ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                document.getElementById('imu-data').textContent = JSON.stringify(data, null, 2);
            };
            imu_ws.onclose = function(event) {
                document.getElementById('imu-data').textContent = 'Connection closed.';
            };
            imu_ws.onerror = function(error) {
                document.getElementById('imu-data').textContent = `Error: ${error.message}`;
            };

            // --- Image WebSocket ---
            const image_ws = new WebSocket(`ws://${window.location.host}/sensors/image`);
            image_ws.binaryType = "blob";
            const imageElement = document.getElementById('camera-image');
            
            image_ws.onmessage = function(event) {

                console.log("Image WebSocket message received:", event.data);

                const url = URL.createObjectURL(event.data);
                imageElement.src = url;
                imageElement.onload = () => {
                    URL.revokeObjectURL(url);
                }
            };
            image_ws.onclose = function(event) {
                console.log('Image WebSocket connection closed.');
            };
            image_ws.onerror = function(error) {
                console.log(`Image WebSocket Error: ${error}`);
            };
        </script>
    </body>
</html>
"""

@app.get("/")
async def get():
    return HTMLResponse(html)


# --- メイン処理 ---
if __name__ == "__main__":
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8000)