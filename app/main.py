#how to run FastAPI
# uvicorn main:app --reload
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image

import asyncio
import cv2
import threading

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

# --- WebSocketクライアント ---
class RobotWebsocketClient:
    def __init__(self, ros_node, robot_id="robot01", server_uri="ws://<基地局PCのIPアドレス>:8000/ws/robot/"):
        self.ros_node = ros_node
        self.uri = f"{server_uri}{robot_id}"
        self.bridge = CvBridge()

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
        """ROSから取得したセンサーデータをサーバーに送信し続ける"""
        while True:
            # IMUデータの送信 (JSON形式)
            with imu_lock:
                imu_msg = latest_imu_msg
            if imu_msg:
                imu_data = imu_to_dict(imu_msg) # imu_to_dict関数は現在のコードから流用
                # データ種別をヘッダーとして付与
                payload = {"type": "imu", "data": imu_data}
                await websocket.send(json.dumps(payload))

            # 画像データの送信 (バイナリ形式)
            cv_image = None
            with image_lock:
                if latest_image_msg:
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(latest_image_msg, desired_encoding='bgr8')
                    except Exception as e:
                        print(f"Image conversion error: {e}")

            if cv_image is not None:
                cv_image_flipped = cv2.flip(cv_image, -1)
                ret, frame = cv2.imencode('.jpg', cv_image_flipped)
                if ret:
                    # バイナリデータの前にヘッダー（文字列）を送信して区別する案もあるが、
                    # まずはバイナリとJSONを混ぜて送信し、フロントエンドで処理する
                    # サーバー側で中継するだけなら問題ない
                    await websocket.send(frame.tobytes())

            await asyncio.sleep(0.033) # 30fps程度

# --- メイン処理 ---
def run_ros_node_thread(cmd_queue):
    rclpy.init()
    ros_node = RosSubscriberNode(cmd_queue)
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    command_queue = queue.Queue()
    
    # ROSノードを別スレッドで実行
    ros_thread = threading.Thread(target=run_ros_node_thread, args=(command_queue,), daemon=True)
    ros_thread.start()

    # ROSノードのインスタンスをWebSocketクライアントに渡す
    # (少し待ってからインスタンスを取得する必要があるかもしれないが、ここでは簡略化)
    # 簡単にするため、グローバル変数や他の方法でROSノードインスタンスを渡す
    
    # ※ この部分は元のコード構造に合わせて調整が必要です。
    # ここでは、RosSubscriberNodeのインスタンスを生成して渡す単純な例を示します。
    # 実際には、スレッド間で安全にオブジェクトを共有する設計にしてください。
    
    # ダミーのrclpy.init/shutdownを避けるため、ROSノードのオブジェクトを直接生成
    # （この構造は要検討。ここではコンセプトを示す）
    rclpy.init()
    temp_node_for_client = RosSubscriberNode(command_queue)

    client = RobotWebsocketClient(ros_node=temp_node_for_client, server_uri="ws://192.168.1.10:8000/ws/robot/") # IPは要変更
    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        print("Client stopped.")