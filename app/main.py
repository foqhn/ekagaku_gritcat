#how to run FastAPI
# uvicorn main:app --reload
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image

import asyncio
import cv2
import threading
import base64

import queue
import json
import websockets

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
oled=OLEDDisplay(font_size=15)
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

                command= command_data.get("command")
        
                left_speed= int(command_data.get("left", 0)) 
                right_speed= int(command_data.get("right", 0)) 

                # モーターが動くコマンドの場合のみリレーをONにする
                if command == "move":
                    if left_speed != 0 or right_speed != 0:
                        lgpio.gpio_write(self.h, self.relay_pin, 1)  # リレーON
                    else:
                        lgpio.gpio_write(self.h, self.relay_pin, 0)  # リレーOFF
                    self.my_motor.move(left_speed, right_speed)
                else:
                    self.my_motor.move(0, 0)  # 安全のため停止
                    lgpio.gpio_write(self.h, self.relay_pin, 0)  # リレーOFF
                    
                self.get_logger().info(f"Motors set to Left: {left_speed}, Right: {right_speed}")

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

# --- WebSocketクライアント ---
class RobotWebsocketClient:
    def __init__(self, ros_node, robot_id="robot01", server_uri="ws://<基地局PCのIPアドレス>:8000/ws/robot/"):
        self.ros_node = ros_node
        self.uri = f"{server_uri}{robot_id}"
        self.bridge = CvBridge()
        oled.display_text([f"id : {robot_id}"])

    async def run(self):
        async with websockets.connect(self.uri) as websocket:
            print(f"Connected to server: {self.uri}")

            # サーバーからのコマンド受信タスク
            listen_task = asyncio.create_task(self.listen_for_commands(websocket))
            # センサーデータの送信タスク
            send_task = asyncio.create_task(self.send_sensor_data(websocket))

            await asyncio.gather(listen_task, send_task)

    async def listen_for_commands(self, websocket):
        """サーバーからコマンドを受信し、ROSノードのキューに入れる"""
        async for message in websocket:
            try:
                command_data = json.loads(message)
                print(f"Received command: {command_data}")
                self.ros_node.command_queue.put(command_data)
            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")

    async def send_sensor_data(self, websocket):
        """ROSから取得したセンサーデータを単一のJSON形式でサーバーに送信し続ける"""
        while True:
            # センサーデータを取得
            with imu_lock:
                imu_msg = latest_imu_msg
            with image_lock:
                image_msg = latest_image_msg

            # 送信するペイロードを作成
            payload = {"type": "sensor_data", "data": {}}
            
            # IMUデータをペイロードに追加
            if imu_msg:
                payload["data"]["imu"] = imu_to_dict(imu_msg)

            # 画像データをBase64エンコードしてペイロードに追加
            if image_msg:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                    cv_image_flipped = cv2.flip(cv_image, -1)
                    ret, frame = cv2.imencode('.jpg', cv_image_flipped)
                    if ret:
                        # Base64エンコードして文字列に変換
                        jpg_as_text = base64.b64encode(frame).decode('utf-8')
                        payload["data"]["image"] = jpg_as_text
                except Exception as e:
                    print(f"Image processing error: {e}")

            # データが何か一つでもあれば送信
            if payload["data"]:
                await websocket.send(json.dumps(payload))

            await asyncio.sleep(0.033) # 30fps程度

# --- メイン処理 ---
def run_ros_spin(node):
    """
    指定されたROSノードのイベントループを実行する。
    """
    print("ROS spin thread started.")
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        # rclpy.shutdown()が呼ばれるとspinは例外を発生させて終了する
        print("ROS spin thread gracefully stopped.")
    finally:
        # スレッドが終了する際にノードを破棄する
        node.cleanup() # GPIOリソースの解放
        node.destroy_node()
        print("ROS Node destroyed.")


# --- メイン処理 ---
if __name__ == "__main__":
    # 変更点 1: プロセスの開始時点でROSを一度だけ初期化する
    rclpy.init()

    command_queue = queue.Queue()
    # 変更点 2: ROSノードのインスタンスをメインスレッドで生成する
    ros_node = RosSubscriberNode(command_queue)
    
    # 変更点 3: ROSのspinを実行するスレッドを開始する
    #           生成したノードのインスタンスを渡す
    ros_thread = threading.Thread(target=run_ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # 変更点 4: WebSocketクライアントにも同じノードのインスタンスを渡す
    #           (主にコマンドキューを共有するために)
    #           IPアドレスは実際の基地局PCのものに変更してください
    client = RobotWebsocketClient(
        ros_node=ros_node, 
        server_uri="ws://192.168.0.101:8000/ws/robot/"
    )

    try:
        # メインスレッドで非同期のWebSocketクライアントを実行
        print("Starting WebSocket client...")
        asyncio.run(client.run())

    except KeyboardInterrupt:
        print("Application stopped by user (Ctrl+C).")
        
    finally:
        # 変更点 5: アプリケーション終了時にROSをクリーンにシャットダウンする
        print("Shutting down rclpy...")
        rclpy.shutdown()
        # ros_threadが終了するのを待つ
        ros_thread.join(timeout=2)
        oled.clear()
        print("Application has exited.")