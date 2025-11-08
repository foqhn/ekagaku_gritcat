#how to run FastAPI
# uvicorn main:app --reload
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image,MagneticField

import asyncio
import cv2
import threading
import base64

import queue
import json
import websockets

from cv_bridge import CvBridge
import numpy as np
import csv
from datetime import datetime

import lgpio
from src.oled import OLEDDisplay
from src.motor_controller import GritMotor
from src.wifi_info import get_wifi_info

import threading
import subprocess
import signal
import os
import time

# --- グローバル変数とロック ---
latest_imu_msg = None
latest_image_msg = None
latest_mag_msg = None
latest_gps_msg = None
latest_system_info = {}

imu_lock = threading.Lock()
image_lock = threading.Lock()
gps_lock=threading.Lock()
system_info_lock=threading.Lock()

command_queue = queue.Queue()
shutdown_event = threading.Event() 
oled=OLEDDisplay(font_size=15)
# --- ROS 2 ノード ---
class RosSubscriberNode(Node):
    def __init__(self,command_queue):
        super().__init__('ros_fastapi_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu, '/bno055/imu', self.imu_callback, 10)
        self.mag_subscription = self.create_subscription(
            MagneticField, '/bno055/mag', self.mag_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()

        self.log_directory = "sensor_logs"  # ログ保存用ディレクトリ
        self.is_logging = False
        self.log_file = None
        self.csv_writer = None
        self.log_timer = None
        self.temp_log_path = None  # 一時ファイルのフルパス
        self.final_log_path = None # 最終的なファイルのフルパス

        self.h= None
        self.my_motor = None

        #subprocess
        self.proc_cam = None 
        self.proc_imu = None
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
    
    def mag_callback(self, msg):
        global latest_mag_msg
        with imu_lock:
            latest_mag_msg = msg

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
                elif command =="sensor":
                    sensor_type=command_data.get("sensor_type")
                    bin=int(command_data.get("bin"))
                    self.sensor_ctl(sensor_type,bin)
                elif command=="log":
                    bin=int(command_data.get("msg"))
                    self.log_data_csv(bin) # ログ管理メソッドを呼び出す


                else:
                    self.my_motor.move(0, 0)  # 安全のため停止
                    lgpio.gpio_write(self.h, self.relay_pin, 0)  # リレーOFF
                    
                self.get_logger().info(f"Motors set to Left: {left_speed}, Right: {right_speed}")

        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
    
        # RosSubscriberNodeクラスのメソッドとして実装
    def sensor_ctl(self, sensor_type, bin):
        """指定されたセンサーのROSノードを起動または停止する"""
        
        # センサータイプに応じて設定を切り替え
        if sensor_type == "cam":
            proc_attr = "proc_cam"
            command = ["ros2", "run", "camera_ros", "camera_node", "--ros-args", "-p", "format:=YUYV"]
            log_prefix = "Camera"
        elif sensor_type == "imu":
            proc_attr = "proc_imu"
            command = ["ros2", "launch", "bno055", "bno055.launch.py"]
            log_prefix = "IMU"
        else:
            self.get_logger().error(f"Unknown sensor type specified: {sensor_type}")
            return

        # getattrを使い、self.proc_cam または self.proc_imu を動的に取得
        current_proc = getattr(self, proc_attr)

        # --- 起動処理 ---
        if bin == 1:
            if current_proc and current_proc.poll() is None:
                self.get_logger().warn(f"{log_prefix} process is already running.")
                return

            self.get_logger().info(f"Starting {log_prefix} with command: {' '.join(command)}")
            new_proc = subprocess.Popen(command, start_new_session=True)
            
            # setattrを使い、self.proc_cam または self.proc_imu に新しいプロセスをセット
            setattr(self, proc_attr, new_proc)
            
            self.get_logger().info(f"{log_prefix} process started with PID: {new_proc.pid} and PGID: {os.getpgid(new_proc.pid)}")

        # --- 停止処理 ---
        else: # bin == 0
            if current_proc and current_proc.poll() is None:
                try:
                    pgid = os.getpgid(current_proc.pid)
                    self.get_logger().info(f"Stopping {log_prefix} process group (PGID: {pgid})...")
                    
                    # SIGINTで穏便な停止を試みる
                    os.killpg(pgid, signal.SIGINT)
                    current_proc.wait(timeout=5)
                    self.get_logger().info(f"{log_prefix} process group gracefully terminated.")

                except subprocess.TimeoutExpired:
                    self.get_logger().warn(f"{log_prefix} process group did not respond. Forcing termination...")
                    # タイムアウトしたらSIGKILLで強制停止
                    os.killpg(pgid, signal.SIGKILL)
                    current_proc.wait()
                    self.get_logger().info(f"{log_prefix} process group killed.")
                
                except ProcessLookupError:
                    self.get_logger().warn(f"{log_prefix} process group already disappeared.")

                finally:
                    # 正常終了でもエラーでも、プロセス変数をクリア
                    setattr(self, proc_attr, None)
            else:
                self.get_logger().info(f"{log_prefix} process is not running or already stopped.")
                setattr(self, proc_attr, None)

    def log_data_csv(self, bin_command):
        """
        サーバーからのコマンドに基づき、ログローテーション方式でCSV記録を開始/停止する。
        """
        # --- ログ開始処理 ---
        if bin_command == 1 and not self.is_logging:
            try:
                # ログディレクトリが存在しなければ作成
                os.makedirs(self.log_directory, exist_ok=True)

                self.is_logging = True
                
                # 日時ベースのファイル名を生成
                base_filename = datetime.now().strftime("%Y%m%d_%H%M%S")
                
                # 一時ファイルと最終ファイルのパスを決定
                self.temp_log_path = os.path.join(self.log_directory, f"{base_filename}_temp.csv")
                self.final_log_path = os.path.join(self.log_directory, f"{base_filename}.csv")
                
                self.get_logger().info(f"Starting new log. Writing to temporary file: {self.temp_log_path}")
                
                # "一時ファイル"を書き込みモードで開く
                self.log_file = open(self.temp_log_path, 'w', newline='')
                self.csv_writer = csv.writer(self.log_file)

                # ヘッダー行を書き込む (内容は以前と同じ)
                header = [
                    "timestamp_sec", "timestamp_nanosec", 
                    "orient_x", "orient_y", "orient_z", "orient_w",
                    "ang_vel_x", "ang_vel_y", "ang_vel_z", 
                    "lin_accel_x", "lin_accel_y", "lin_accel_z",
                    "mag_x", "mag_y", "mag_z", 
                    "temperature_celsius", 
                    "wifi_ssid", "wifi_signal_strength"
                ]
                self.csv_writer.writerow(header)

                # 定期書き込みタイマーを開始
                self.log_timer = self.create_timer(0.1, self._write_log_callback)

            except (IOError, OSError) as e:
                self.get_logger().error(f"Failed to start logging: {e}")
                self.is_logging = False # 失敗した場合はステータスを戻す

        # --- ログ停止 & ファイル確定処理 ---
        elif bin_command == 0 and self.is_logging:
            self.is_logging = False
            
            # 1. タイマーを停止
            if self.log_timer:
                self.log_timer.cancel()
                self.log_timer = None
            
            # 2. ファイルハンドルを閉じる (重要！)
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.csv_writer = None
            
            # 3. 一時ファイルを最終ファイル名にリネーム
            try:
                if self.temp_log_path and os.path.exists(self.temp_log_path):
                    os.rename(self.temp_log_path, self.final_log_path)
                    self.get_logger().info(f"Log file finalized: {self.final_log_path}")
                else:
                    self.get_logger().warn("Temporary log file not found, nothing to finalize.")
            except (IOError, OSError) as e:
                self.get_logger().error(f"Failed to rename log file: {e}")

            # 4. 状態変数をリセット
            self.temp_log_path = None
            self.final_log_path = None
        
        
    def _write_log_callback(self):
        """
        タイマーによって定期的に呼び出され、センサーデータをCSVに書き込む。
        """
        # (このメソッドは、現在開いているファイルに書き込むだけなので変更不要)
        if not self.is_logging or not self.csv_writer:
            return
        # ... (データの取得と書き込み処理は以前のまま)
        # (ヘッダーに合わせてデータを並べる部分を確認してください)
        with imu_lock:
            imu_msg = latest_imu_msg
            mag_msg = latest_mag_msg
        with gps_lock:
            gps_msg = latest_gps_msg
        with system_info_lock:
            system_info = latest_system_info.copy()
        if not imu_msg:
            return
        row = [
            imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec, 
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w,
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, 
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
        ]
        row.extend([mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z] if mag_msg else [None, None, None])
        row.append(gps_msg.fix if gps_msg else None)
        row.append(system_info.get('wifi_ssid'))
        row.append(system_info.get('wifi_strength'))
        self.csv_writer.writerow(row)

    def cleanup(self):
        """プログラム終了時にリソースを安全に解放する"""
        if self.is_logging:
            self.get_logger().info("Finalizing active log due to cleanup...")
            self.log_data_csv(0) # bin=0 として停止&リネーム処理を呼び出す

        if self.h and self.my_motor:
            self.get_logger().info("Cleaning up GPIO resources...")
            self.my_motor.move(0, 0) # 安全のためモーターを停止
            self.my_motor.cleanup() # モーターリソースを解放
            lgpio.gpio_write(self.h, self.relay_pin, 0) # リレーをOFF
            lgpio.gpiochip_close(self.h) # GPIOハンドルを解放
            

# def run_ros_node(cmd_queue, shutdown_evt):
#     print("ROS Node Thread Started")
#     rclpy.init()
#     ros_node = RosSubscriberNode(cmd_queue)
    
#     try:
#         # ros_nodeの初期化に失敗した場合は、spinを呼ばずに終了
#         if ros_node.my_motor:
#             # shutdown_evtがセットされるまでループを続ける
#             while not shutdown_evt.is_set():
#                 # 0.1秒のタイムアウト付きでROSのイベントを一度だけ処理する
#                 # これにより、ループがCPUを100%消費するのを防ぎ、
#                 # shutdown_evtをチェックする機会を定期的に作る
#                 rclpy.spin_once(ros_node, timeout_sec=0.1)
#         else:
#             print("ERROR: ROS Node could not start due to motor initialization failure.")
    
#     except Exception as e:
#         # 通常はここに到達しないはずだが、念のため
#         print(f"An exception occurred in ROS thread: {e}")
        
#     finally:
#         # ループが終了したら（つまりシャットダウンが要求されたら）、後処理を実行
#         print("ROS Node Thread is shutting down...")
#         ros_node.cleanup()
#         ros_node.destroy_node()
#         rclpy.shutdown()
#         print("ROS Node Thread has been shut down successfully.")



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
        self.ssid, self.strength = get_wifi_info()
        text_lines = []
        text_lines.append(f"id :  {robot_id}")
        text_lines.append("")
        if self.ssid and self.strength:
            text_lines.append(f"SSID :  {self.ssid}")

        else:
            text_lines.append("Wi-Fi Not Connected")
        oled.display_text(text_lines, start_x=5, start_y=5, line_spacing=12)

    async def run(self):
        async with websockets.connect(self.uri) as websocket:
            print(f"Connected to server: {self.uri}")

            # サーバーからのコマンド受信タスク
            listen_task = asyncio.create_task(self.listen_for_commands(websocket))
            # センサーデータの送信タスク
            send_task = asyncio.create_task(self.send_sensor_data(websocket))

            await asyncio.gather(listen_task, send_task)

    async def listen_for_commands(self, websocket):
        """サーバーからコマンドを受信し、内容に応じて処理を振り分ける"""
        async for message in websocket:
            try:
                command_data = json.loads(message)
                command = command_data.get("command")
                print(f"Received command: {command_data}")

                # --- ここからが変更点 ---
                if command == "list_log_files":
                    # ファイル一覧取得コマンドを処理
                    await self.handle_list_log_files(websocket)

                elif command == "get_log_file":
                    # ファイル取得コマンドを処理
                    filename = command_data.get("filename")
                    if filename:
                        await self.handle_get_log_file(websocket, filename)
                    else:
                        print("Error: 'filename' not provided for get_log_file command.")
                        # エラーをサーバーに通知するのも良いでしょう
                        await websocket.send(json.dumps({
                            "type": "error",
                            "message": "'filename' is required for get_log_file."
                        }))
                
                else:
                    # 従来通りのコマンドはROSノードのキューに入れる
                    self.ros_node.command_queue.put(command_data)

            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
            except Exception as e:
                print(f"Error processing command: {e}")

    async def send_sensor_data(self, websocket):
        """ROSから取得したセンサーデータを単一のJSON形式でサーバーに送信し続ける"""
        while True:
            # センサーデータを取得
            with imu_lock:
                imu_msg = latest_imu_msg
                mag_msg = latest_mag_msg
            with image_lock:
                image_msg = latest_image_msg

            # 送信するペイロードを作成
            payload = {"type": "sensor_data", "data": {}}
            
            # IMUデータをペイロードに追加
            if imu_msg:
                payload["data"]["imu"] = imu_to_dict(imu_msg)
            # 磁気データをペイロードに追加
            if mag_msg:
                payload["data"]["mag"] = {
                    'header': {
                        'stamp': {'sec': mag_msg.header.stamp.sec, 'nanosec': mag_msg.header.stamp.nanosec},
                        'frame_id': mag_msg.header.frame_id
                    },
                    'magnetic_field': {
                        'x': mag_msg.magnetic_field.x,
                        'y': mag_msg.magnetic_field.y,
                        'z': mag_msg.magnetic_field.z
                    }
                }
            ssid, strength = get_wifi_info()
            with system_info_lock:
                if ssid and strength:
                    latest_system_info['wifi_ssid'] = ssid
                    latest_system_info['wifi_strength'] = strength

            if ssid and strength:
                payload["data"]["wifi"] = {
                    "ssid": ssid,
                    "signal_strength": strength
                }
            else:
                payload["data"]["wifi"] = {
                    "ssid": None,
                    "signal_strength": None
                }

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

            #TODO:FPSをサーバー側からいじれるようにしたい．
            await asyncio.sleep(0.033) # 30fps程度 

    async def handle_list_log_files(self, websocket):
        """ログディレクトリ内のCSVファイル一覧をサーバーに送信する"""
        try:
            log_dir = self.ros_node.log_directory  # ROSノードが持つログディレクトリのパスを参照
            if not os.path.isdir(log_dir):
                await websocket.send(json.dumps({
                    "type": "error", 
                    "message": f"Log directory not found: {log_dir}"
                }))
                return

            # ディレクトリ内のCSVファイルのみをリストアップ
            files = [
                f for f in os.listdir(log_dir) 
                if f.endswith('.csv') and not f.endswith('_temp.csv')
            ]
            
            # サーバーに応答を送信
            response = {
                "type": "log_file_list",
                "files": files
            }
            await websocket.send(json.dumps(response))
            print(f"Sent log file list: {files}")

        except Exception as e:
            print(f"Error listing log files: {e}")
            await websocket.send(json.dumps({"type": "error", "message": str(e)}))

    async def handle_get_log_file(self, websocket, filename):
        """指定されたログファイルを読み込み、その内容をサーバーに送信する"""
        try:
            # --- セキュリティ対策 ---
            # ファイル名にディレクトリトラバーサル攻撃の可能性がないかチェック
            if ".." in filename or filename.startswith("/"):
                raise ValueError("Invalid filename specified.")
            
            log_dir = self.ros_node.log_directory
            file_path = os.path.join(log_dir, filename)

            if os.path.exists(file_path):
                # ファイルを読み込む
                with open(file_path, 'r', encoding='utf-8') as f:
                    file_content = f.read()
                
                # サーバーに応答を送信
                response = {
                    "type": "log_file_content",
                    "filename": filename,
                    "data": file_content
                }
                await websocket.send(json.dumps(response))
                print(f"Sent file content for: {filename}")
            
            else:
                # ファイルが存在しない場合のエラー応答
                response = {
                    "type": "error",
                    "message": "File not found.",
                    "filename": filename
                }
                await websocket.send(json.dumps(response))
                print(f"File not found: {file_path}")

        except Exception as e:
            print(f"Error getting log file '{filename}': {e}")
            await websocket.send(json.dumps({
                "type": "error", 
                "message": str(e),
                "filename": filename
            }))

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
        server_uri="ws://192.168.137.1:8000/ws/robot/"
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