#how to run FastAPI
# uvicorn main:app --reload
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image

import asyncio
import cv2
import threading
import uvicorn
import queue
from fastapi import FastAPI ,Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates

from starlette.websockets import WebSocket, WebSocketDisconnect

from cv_bridge import CvBridge
import numpy as np

import lgpio
from src.oled import OLEDDisplay
from src.motor_controller import GritMotor

import threading

# --- グローバル変数とロック ---
latest_imu_msg = None
latest_image_msg = None
imu_lock = threading.Lock()
image_lock = threading.Lock()
command_queue = queue.Queue()
shutdown_event = threading.Event() 
# --- ROS 2 ノード ---
class RosSubscriberNode(Node):
    def __init__(self,command_queue):
        super().__init__('ros_fastapi_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu, '/bno055/imu', self.imu_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.h= None
        self.my_motor = None
        try:
            self.h= lgpio.gpiochip_open(0)
            self.relay_pin = 17
            lgpio.gpio_claim_output(self.h, self.relay_pin)
            self.my_motor = GritMotor(self.h)

            lgpio.gpio_write(self.h, self.relay_pin, 0)  # リレーOFFで初期化
            self.get_logger().info('Motor controller initialized successfully.')
        except Exception as e:  
            self.get_logger().error(f'Error initializing motor controller: {e}')


        self.command_queue = command_queue
        self.command_timer = self.create_timer(0.1, self.process_commands)
        self.get_logger().info('ROS Subscriber Node has been started.')

    def imu_callback(self, msg):
        global latest_imu_msg
        with imu_lock:
            latest_imu_msg = msg

    def image_callback(self, msg):
        global latest_image_msg
        with image_lock:
            latest_image_msg = msg
        #self.get_logger().info('<<<<< Image data received by ROS node! >>>>>')
    def process_commands(self):
        """キューを監視し、コマンドに応じてモーターを直接制御する"""
        # モーターが正常に初期化されていない場合は何もしない
        if not self.my_motor:
            return
        try:
            while not self.command_queue.empty():
                command_data = self.command_queue.get_nowait()
                self.get_logger().info(f"Processing command: {command_data}")

                command = command_data.get("command")
                # クライアントから速度が指定されなければ、デフォルト50に設定
                speed = int(command_data.get("speed", 10)) 

                # モーターが動くコマンドの場合のみリレーをONにする
                if command in ["forward", "backward", "turn_left", "turn_right"]:
                    lgpio.gpio_write(self.h, self.relay_pin, 1)
                    

                if command == "forward":
                    self.my_motor.move(speed, speed)
                elif command == "backward":
                    self.my_motor.move(-speed, -speed)
                elif command == "turn_left":
                    # その場で左回転
                    self.my_motor.move(-speed, speed)
                elif command == "turn_right":
                    # その場で右回転
                    self.my_motor.move(speed, -speed)
                elif command == "stop":
                    self.my_motor.move(0, 0)
                    # 停止時にリレーをOFFにする
                    lgpio.gpio_write(self.h, self.relay_pin, 0)
        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
    
    def cleanup(self):
        """プログラム終了時にリソースを安全に解放する"""
        if self.h and self.my_motor:
            self.get_logger().info("Cleaning up GPIO resources...")
            self.my_motor.move(0, 0) # 安全のためモーターを停止
            self.my_motor.cleanup() # モーターリソースを解放
            lgpio.gpio_write(self.h, self.relay_pin, 0) # リレーをOFF
            lgpio.gpiochip_close(self.h) # GPIOハンドルを解放
            

def run_ros_node(cmd_queue, shutdown_evt):
    print("ROS Node Thread Started")
    rclpy.init()
    ros_node = RosSubscriberNode(cmd_queue)
    
    try:
        # ros_nodeの初期化に失敗した場合は、spinを呼ばずに終了
        if ros_node.my_motor:
            # shutdown_evtがセットされるまでループを続ける
            while not shutdown_evt.is_set():
                # 0.1秒のタイムアウト付きでROSのイベントを一度だけ処理する
                # これにより、ループがCPUを100%消費するのを防ぎ、
                # shutdown_evtをチェックする機会を定期的に作る
                rclpy.spin_once(ros_node, timeout_sec=0.1)
        else:
            print("ERROR: ROS Node could not start due to motor initialization failure.")
    
    except Exception as e:
        # 通常はここに到達しないはずだが、念のため
        print(f"An exception occurred in ROS thread: {e}")
        
    finally:
        # ループが終了したら（つまりシャットダウンが要求されたら）、後処理を実行
        print("ROS Node Thread is shutting down...")
        ros_node.cleanup()
        ros_node.destroy_node()
        rclpy.shutdown()
        print("ROS Node Thread has been shut down successfully.")



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

# --- FastAPI アプリケーション ---
app = FastAPI()
template=Jinja2Templates(directory="app/templates")
@app.get("/")
async def index(request: Request):
    return template.TemplateResponse("index.html", {"request": request})
@app.on_event("shutdown")
def shutdown_handler():
    print("FastAPI is shutting down. Signaling ROS thread to exit.")
    shutdown_event.set()


@app.websocket("/sensors/imu")
async def websocket_imu_endpoint(websocket: WebSocket):
    print("Loaded")
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
                    cv_image_flipped = cv2.flip(cv_image, -1)
                    ret, frame = cv2.imencode('.jpg', cv_image_flipped)
                    if ret:
                        image_to_send = frame.tobytes()
                    else:
                        print("!!! cv2.imencode failed !!!")
                except Exception as e:
                    print(f"!!! Error in cv2.imencode: {e} !!!")


            if image_to_send:
                #print(f">>>>> Sending image data via WebSocket (size: {len(image_to_send)} bytes)")
                await websocket.send_bytes(image_to_send)
                
            await asyncio.sleep(0.033)
            
    except WebSocketDisconnect:
        print("Image client disconnected")
    except Exception as e:
        print(f"An unexpected error occurred in Image websocket: {e}")


@app.websocket("/control")
async def websocket_control_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Control client connected.")
    try:
        while True:
            # クライアントからJSON形式でデータを受信
            data = await websocket.receive_json()
            print(f"Received command from client: {data}")
            
            # 受信したコマンドをグローバルなキューに入れる
            command_queue.put(data)
            
    except WebSocketDisconnect:
        print("Control client disconnected.")
    except Exception as e:
        print(f"An error occurred in control websocket: {e}")



# --- メイン処理 ---
if __name__ == "__main__":
    ros_thread = threading.Thread(target=run_ros_node,args=(command_queue,shutdown_event), daemon=True)
    ros_thread.start()
    uvicorn.run(app, host="192.168.0.100", port=8000)
    ros_thread.join(timeout=5)
    print("Main application has exited.")