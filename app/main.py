# how to run FastAPI
# uvicorn main:app --reload

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image, MagneticField, NavSatFix, NavSatStatus

import ctypes
import asyncio
import cv2
import threading
import base64

import queue
import json
import websockets

from cv_bridge import CvBridge
import numpy as np
import math
import csv
from datetime import datetime

import lgpio
from src.oled import OLEDDisplay
from src.motor_controller import GritMotor
from src.system_info import get_wifi_info, get_cpu_temperature
from src.i2c_utils import scan_i2c_bus
from src.config_manager import ConfigManager
from src.bme280_lgpio import BME280
from src.mcp23017 import MCP23017 

import subprocess
import signal
import os
import sys
import time
import atexit  
import shutil

# ==========================================================
# グローバル変数と排他制御 (Locks)
# ==========================================================
# ROSスレッドとメインスレッド/WebSocketスレッド間でデータを共有するための変数
latest_imu_msg = None
latest_image_msg = None
latest_mag_msg = None
latest_gps_msg = None
latest_system_info = {}
latest_bme_data = None

# 画像処理結果（デバッグ用）の共有変数
latest_debug_image = None
last_debug_update_time = 0.0

# スレッドセーフなアクセスのためのロックオブジェクト
imu_lock = threading.Lock()
image_lock = threading.Lock()
gps_lock = threading.Lock()
system_info_lock = threading.Lock()
bme_lock = threading.Lock()
debug_image_lock = threading.Lock() 

command_queue = queue.Queue()
shutdown_event = threading.Event() 

# OLEDディスプレイの初期化
oled = OLEDDisplay(font_size=15)

# 設定をロード
config = ConfigManager.load_config()
CURRENT_ROBOT_ID = config.get("robot_id", "robot_unknown")
SERVER_IP = config.get("server_ip", "192.168.11.14") # IPもConfig管理する場合
SERVER_PORT = config.get("server_port", 8000)

# ==========================================================
# ヘルパー関数
# ==========================================================
def show_status_display():
    """
    待機画面（ロボットIDとWi-Fi接続情報）をOLEDに表示する関数。
    プログラム停止時や起動時に呼び出される。
    """
    try:
        ssid, strength = get_wifi_info()
        
        text_lines = []
        text_lines.append(f"ID : {CURRENT_ROBOT_ID}")
        text_lines.append("") # 空行
        
        if ssid:
            text_lines.append(f"Wi-Fi: {ssid[:12]}") 
            text_lines.append(f"Signal: {strength}%")
        else:
            text_lines.append("Wi-Fi: Disconnected")
            
        oled.display_text(text_lines, start_x=0, start_y=0, line_spacing=12)
    except Exception as e:
        print(f"OLED Error: {e}")

def raise_keyboard_interrupt(thread_obj):
    """
    指定されたスレッドに対して非同期に KeyboardInterrupt 例外を送出する。
    ユーザープログラムの強制停止に使用。
    """
    if not thread_obj.is_alive():
        return
    
    tid = ctypes.c_long(thread_obj.ident)
    ex_type = ctypes.py_object(KeyboardInterrupt) 
    
    # Python C APIを利用して例外をセット
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ex_type)
    if res == 0:
        print("Error: Invalid thread ID")
    elif res > 1:
        # 意図しない影響が出た場合は取り消し
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)

def force_kill_os_process(pattern):
    """
    指定されたパターンに一致するOSプロセスを強制終了(SIGKILL)する。
    ゾンビプロセスのクリーンアップ用。
    """
    try:
        if shutil.which("pkill"):
            subprocess.run(["pkill", "-f", "-9", pattern], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"Failed to force kill process pattern '{pattern}': {e}")


# ==========================================================
# RobotController クラス
# ユーザープログラム(user_program.py)から利用されるAPIを提供する
# ==========================================================
class RobotController:
    def __init__(self, ros_node, stop_event):
        self.ros_node = ros_node
        self._stop_event = stop_event 

        # --- 画像処理用の内部状態 ---
        self._cv_image = None       # 現在処理中の画像 (OpenCV BGR形式)
        self._roi_offset_x = 0      # ROI（関心領域）によるX座標のズレ
        self._roi_offset_y = 0      # ROIによるY座標のズレ
        self._image_width = 0
        self._image_height = 0

    def _check_stop(self):
        """
        各メソッドの実行前に呼び出し、停止フラグが立っていたら
        例外を投げてスクリプトを即座に中断させる安全装置。
        """
        if self._stop_event.is_set():
            raise InterruptedError("Program stopped by user.")

    # ----------------------------------------------------------
    #  モーター & 基本制御
    # ----------------------------------------------------------
    def move(self, left, right, duration=None):
        """左右モーターの速度制御 (-100 ~ 100)"""
        self._check_stop()
        cmd = {"command": "move", "left": int(left), "right": int(right)}
        self.ros_node.command_queue.put(cmd)

        if duration is not None:
            self.sleep(duration)
            self.stop()

    def stop(self):
        """モーター停止"""
        cmd = {"command": "move", "left": 0, "right": 0}
        self.ros_node.command_queue.put(cmd)

    def sleep(self, seconds):
        """
        中断可能なスリープ処理。
        time.sleepをそのまま使うと停止指令を受け付けなくなるため、細切れに待機する。
        """
        start_time = time.time()
        while time.time() - start_time < seconds:
            self._check_stop()
            time.sleep(0.1) 

    # ----------------------------------------------------------
    #  センサー取得
    # ----------------------------------------------------------
    def get_sensor(self, sensor_type):
        """
        各種センサーの最新値を取得して辞書形式で返す。
        sensor_type: 'compass', 'imu', 'mag', 'gps', 'bme280', 'wifi', 'battery'
        """
        self._check_stop()
        
        if sensor_type == 'compass':
            # IMUのクォータニオンからヨー角（方位）を計算
            with imu_lock:
                imu_msg = latest_imu_msg
            if not imu_msg: return {'heading': 0.0}
            try:
                q = imu_msg.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                # 北を0度として時計回りの角度(0-360)に変換
                compass_deg = (90.0 - math.degrees(yaw)) % 360.0
                return {'heading': float(compass_deg)}
            except Exception:
                return {'heading': 0.0}

        elif sensor_type == 'imu':
            data = None
            with imu_lock:
                if latest_imu_msg: data = imu_to_dict(latest_imu_msg)
            if data: return data
            # データがない場合のデフォルト値
            return {
                'linear_acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            }
        
        # ... 他のセンサー処理 (mag, gps, bme280, wifi, battery) ...
        elif sensor_type == 'mag':
            data = None
            with imu_lock:
                msg = latest_mag_msg
                if msg:
                    data = {'magnetic_field': {'x': msg.magnetic_field.x, 'y': msg.magnetic_field.y, 'z': msg.magnetic_field.z}}
            if data: return data
            return {'magnetic_field': {'x': 0.0, 'y': 0.0, 'z': 0.0}}

        elif sensor_type == 'gps':
            data = None
            with gps_lock:
                if latest_gps_msg: data = gps_to_dict(latest_gps_msg)
            if data: return data
            return {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0, 'status': {'status': 0}}

        elif sensor_type == 'bme280':
            data = None
            with bme_lock:
                if latest_bme_data: data = latest_bme_data.copy()
            if data:
                return {
                    'temperature_celsius': data.get('temperature', 0.0),
                    'humidity_percent': data.get('humidity', 0.0),
                    'pressure_hpa': data.get('pressure', 0.0)
                }
            return {'temperature_celsius': 0.0, 'humidity_percent': 0.0, 'pressure_hpa': 0.0}

        elif sensor_type == 'wifi':
            ssid, rssi = get_wifi_info()
            if rssi is None: rssi = 0
            return {'rssi': rssi, 'ssid': ssid}
        
        elif sensor_type == 'battery':
            # 現状はダミー値を返す
            return {'voltage': 0.0}

        else:
            self.print(f"Warning: Unknown sensor type '{sensor_type}'")
            return {}

    # ----------------------------------------------------------
    #  IO / Display (MCP23017 & OLED)
    # ----------------------------------------------------------
    def set_pin_mode(self, pin, mode):
        """GPIOエキスパンダのピンモード設定 (mode: 'in' or 'out')"""
        self._check_stop()
        if not self.ros_node.mcp: return
        m_val = MCP23017.INPUT if mode == 'in' else MCP23017.OUTPUT
        # 入力モード時はプルアップ有効化
        pull_up = True if mode == 'in' else False
        with self.ros_node.mcp_lock:
            try:
                self.ros_node.mcp.setup(pin, m_val, pull_up=pull_up)
            except Exception as e:
                self.print(f"IO Setup Error: {e}")

    def digital_write(self, pin, value):
        """デジタル出力"""
        self._check_stop()
        if not self.ros_node.mcp: return
        val = 1 if value else 0
        with self.ros_node.mcp_lock:
            try:
                self.ros_node.mcp.output(pin, val)
            except Exception as e:
                self.print(f"IO Write Error: {e}")

    def digital_read(self, pin):
        """デジタル入力"""
        self._check_stop()
        if not self.ros_node.mcp: return 0
        with self.ros_node.mcp_lock:
            try:
                return self.ros_node.mcp.input(pin)
            except Exception as e:
                self.print(f"IO Read Error: {e}")
                return 0

    def buzzer(self, enable):
        """ブザー制御（MCP23017のPin 0に接続されている想定）"""
        self.digital_write(0, 1 if enable else 0)

    def print_display(self, message):
        """OLEDディスプレイにメッセージを表示"""
        self._check_stop()
        try:
            lines = []
            if isinstance(message, list):
                lines = [str(x) for x in message]
            else:
                lines = str(message).split('\n')
            oled.display_text(lines, start_x=0, start_y=0, line_spacing=15)
        except Exception as e:
            print(f"[OLED Error]: {e}")

    def print(self, text):
        """コンソールへのログ出力（プレフィックス付き）"""
        print(f"[Robot]: {text}")

    # ----------------------------------------------------------
    #  画像処理 (OpenCV)
    # ----------------------------------------------------------
    def capture_image(self):
        """
        カメラから最新の画像を取得し、内部変数 _cv_image に保存する。
        【重要】画像の上下反転(-1)を行い、正立画像として保持する。
        """
        self._check_stop()
        global latest_image_msg
        
        with image_lock:
            if latest_image_msg is None:
                self.print("Warning: No camera image received yet.")
                self._cv_image = None
                return False
            
            try:
                # ROSメッセージ -> OpenCV画像変換
                cv_img = self.ros_node.bridge.imgmsg_to_cv2(latest_image_msg, desired_encoding='bgr8')
                # カメラ取り付け向きに合わせて正立にする (flip code -1: 両軸反転)
                cv_img = cv2.flip(cv_img, -1)
                
                self._cv_image = cv_img
                self._image_height, self._image_width = cv_img.shape[:2]
                
                # 画像を新しく取得したのでROIオフセットをリセット
                self._roi_offset_x = 0
                self._roi_offset_y = 0
                return True
            except Exception as e:
                self.print(f"Image capture error: {e}")
                self._cv_image = None
                return False

    def get_image_size(self):
        if self._cv_image is None: return (0, 0)
        h, w = self._cv_image.shape[:2]
        return (w, h)

    def set_roi(self, x, y, w, h):
        """
        画像を関心領域(ROI)で切り抜く。
        以降の画像処理はこの切り抜かれた領域に対して行われる。
        座標のオフセットを記憶し、後でグローバル座標に戻せるようにする。
        """
        self._check_stop()
        if self._cv_image is None: return

        current_h, current_w = self._cv_image.shape[:2]
        # 範囲外アクセスのガード
        x = max(0, min(x, current_w - 1))
        y = max(0, min(y, current_h - 1))
        w = max(1, min(w, current_w - x))
        h = max(1, min(h, current_h - y))

        self._cv_image = self._cv_image[y:y+h, x:x+w]
        
        # オフセットを累積（切り抜いた分、原点がずれるのを補正）
        self._roi_offset_x += x
        self._roi_offset_y += y

    def enhance_contrast(self, clip_limit=2.0):
        """コントラスト強調 (CLAHE)"""
        self._check_stop()
        if self._cv_image is None: return
        try:
            lab = cv2.cvtColor(self._cv_image, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8,8))
            l = clahe.apply(l)
            lab = cv2.merge((l, a, b))
            self._cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        except Exception as e:
            self.print(f"Contrast enhance error: {e}")

    def apply_morphology(self, operation, kernel_size=5):
        """モルフォロジー演算 (ノイズ除去等)"""
        self._check_stop()
        if self._cv_image is None: return
        try:
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            op = cv2.MORPH_OPEN
            if operation == 'close': op = cv2.MORPH_CLOSE
            elif operation == 'erode': op = cv2.MORPH_ERODE
            elif operation == 'dilate': op = cv2.MORPH_DILATE
            self._cv_image = cv2.morphologyEx(self._cv_image, op, kernel)
        except Exception as e:
            self.print(f"Morphology error: {e}")

    def detect_color_centroid(self, color_space, min_vals, max_vals):
        """
        指定色の重心を検出する。
        戻り値の(x, y)は、ROI切り抜き前の「画面全体の座標系」で返す。
        """
        self._check_stop()
        if self._cv_image is None:
            return {'exists': False, 'x': 0, 'y': 0, 'area': 0}

        try:
            target_img = None
            if color_space.upper() == 'HSV':
                target_img = cv2.cvtColor(self._cv_image, cv2.COLOR_BGR2HSV)
            elif color_space.upper() == 'HSL':
                target_img = cv2.cvtColor(self._cv_image, cv2.COLOR_BGR2HLS)
            else:
                target_img = self._cv_image

            lower = np.array(min_vals, dtype=np.uint8)
            upper = np.array(max_vals, dtype=np.uint8)
            
            mask = cv2.inRange(target_img, lower, upper)
            M = cv2.moments(mask)
            area = M['m00']
            
            if area > 0:
                cx = int(M['m10'] / area)
                cy = int(M['m01'] / area)
                
                # 現在のROI座標系からグローバル座標系に変換
                global_x = cx + self._roi_offset_x
                global_y = cy + self._roi_offset_y
                
                return {'exists': True, 'x': global_x, 'y': global_y, 'area': int(area)}
            else:
                return {'exists': False, 'x': 0, 'y': 0, 'area': 0}

        except Exception as e:
            self.print(f"Color detect error: {e}")
            return {'exists': False, 'x': 0, 'y': 0, 'area': 0}

    # --- 視覚化・デバッグ用メソッド ---

    def draw_marker(self, x, y, color=(0, 255, 0), size=15):
        """
        画面上の指定座標(グローバル座標)にマーカーを描画する。
        内部でROI座標系に変換して描画を行う。
        """
        self._check_stop()
        if self._cv_image is None: return

        # グローバル座標 -> 現在の画像（ROI）内座標
        lx = int(x - self._roi_offset_x)
        ly = int(y - self._roi_offset_y)
        
        h, w = self._cv_image.shape[:2]
        # 描画範囲内なら描画
        if -size < lx < w + size and -size < ly < h + size:
            cv2.drawMarker(self._cv_image, (lx, ly), color, 
                           markerType=cv2.MARKER_CROSS, markerSize=size, thickness=2)

    def draw_rect(self, x, y, w, h, color=(0, 255, 0), thickness=2):
        """指定領域(グローバル座標)に矩形を描画する"""
        self._check_stop()
        if self._cv_image is None: return

        lx = int(x - self._roi_offset_x)
        ly = int(y - self._roi_offset_y)
        cv2.rectangle(self._cv_image, (lx, ly), (lx+w, ly+h), color, thickness)

    def show_image(self):
        """
        現在の処理画像をWeb画面への配信用として登録する。
        これを呼ぶと、フロントエンドには生のカメラ映像の代わりに
        この時点の画像（加工・描画済み）が優先して表示される。
        """
        self._check_stop()
        if self._cv_image is None: return

        global latest_debug_image, last_debug_update_time
        with debug_image_lock:
            latest_debug_image = self._cv_image.copy()
            last_debug_update_time = time.time()


# ==========================================================
# ScriptManager クラス
# ユーザープログラムの実行・停止・保存・物理ボタン監視
# ==========================================================
class ScriptManager:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.script_path = "user_program.py"  
        self.stop_event = threading.Event()
        self.execution_thread = None
        self.ros_node.get_logger().info("ScriptManager initialized (Watching MCP23017 Pin 8 for button)")

    def save_code(self, code_str):
        """受信したコードをファイルに保存"""
        try:
            normalized_code = code_str.replace('\r\n', '\n').replace('\r', '\n')
            with open(self.script_path, "w", encoding="utf-8") as f:
                f.write(normalized_code)
            print(f"Code saved to {self.script_path}")
            return True
        except Exception as e:
            print(f"Save error: {e}")
            return False

    def start_program(self):
        """ユーザープログラムを別スレッドで実行開始"""
        if self.execution_thread and self.execution_thread.is_alive():
            print("Program is already running.")
            return

        if not os.path.exists(self.script_path):
            print("No program file found.")
            return

        print(">>> Starting User Program >>>")
        self.stop_event.clear()
        
        with open(self.script_path, "r", encoding="utf-8") as f:
            code_str = f.read()

        # daemon=Trueにすることでメインプロセス終了時に道連れにする
        self.execution_thread = threading.Thread(
            target=self._run_script_thread, 
            args=(code_str,), 
            daemon=True
        )
        self.execution_thread.start()

    def stop_program(self):
        """実行中のプログラムを強制停止"""
        if self.execution_thread and self.execution_thread.is_alive():
            print(">>> Sending KeyboardInterrupt to User Script... >>>")
            self.stop_event.set()
            # スレッドに例外を注入して中断させる
            raise_keyboard_interrupt(self.execution_thread)
            self.execution_thread.join(timeout=2)
            
            # それでも止まらない場合の最終手段
            if self.execution_thread.is_alive():
                print("Warning: Script is stubborn. SystemExit injection.")
                self._inject_system_exit() 

        # 安全のためモーター停止コマンドを送信
        self.ros_node.command_queue.put({"command": "move", "left": 0, "right": 0})
        self.stop_event.clear()

    def _run_script_thread(self, code_str):
        """実際にユーザースクリプトを実行するスレッド本体"""
        robot = RobotController(self.ros_node, self.stop_event)
        
        # ユーザープログラム内で利用可能な変数・モジュールを定義
        local_scope = {
            "robot": robot,
            "time": time,
            "np": np,
            "print": print,
            "math": __import__("math") 
        }
        try:
            print(">>> User Script Started >>>")
            oled.display_text(["", "   Program", "   Running...", ""], start_x=5, start_y=5, line_spacing=15)
            # 文字列として渡されたPythonコードを実行
            exec(code_str, {}, local_scope)
            print("<<< User Script Finished Normally <<<")

        except KeyboardInterrupt:
            print("\n!!! User Script Interrupted by System (Stop Command) !!!")
            oled.display_text(["", "   STOPPED", "", ""], start_x=5, start_y=5)
            time.sleep(1.0) 

        except SyntaxError as e:
            print(f"!!! Syntax Error in User Script: line {e.lineno} !!!\n{e}")

        except Exception as e:
            print(f"!!! Runtime Error in User Script: {e} !!!")
            import traceback
            traceback.print_exc()

        finally:
            print("--- Safety Cleanup: Stopping Motors ---")
            robot.stop()
            show_status_display()

    def check_button(self):
        """MCP23017のPin 8に接続されたボタンを監視し、プログラムの開始/停止をトグルする"""
        if not self.ros_node.mcp: return
        try:
            is_pressed = False
            with self.ros_node.mcp_lock:
                # 0が押下状態 (Pull-up)
                if self.ros_node.mcp.input(8) == 0:
                    is_pressed = True

            if is_pressed:
                # チャタリング防止
                time.sleep(0.05)
                still_pressed = False
                with self.ros_node.mcp_lock:
                    if self.ros_node.mcp.input(8) == 0:
                        still_pressed = True
                
                if still_pressed:
                    if self.execution_thread and self.execution_thread.is_alive():
                        print("Button: Stop Program")
                        self.stop_program()
                        self._wait_for_release(8)
                    else:
                        print("Button: Start Program")
                        self.start_program()
                        self._wait_for_release(8)
        except Exception as e:
            print(f"Button Check Error: {e}")

    def _wait_for_release(self, pin):
        """ボタンが離されるまで待機"""
        while True:
            val = 1
            try:
                with self.ros_node.mcp_lock:
                    val = self.ros_node.mcp.input(pin)
            except: pass
            if val == 1: break
            time.sleep(0.1)


# ==========================================================
# ROS 2 ノード
# センサー購読、ハードウェア制御、外部プロセス管理
# ==========================================================
class RosSubscriberNode(Node):
    def __init__(self, command_queue):
        super().__init__('ros_fastapi_subscriber')
        
        # --- センサーのサブスクライバ設定 ---
        self.imu_subscription = self.create_subscription(
            Imu, '/bno055/imu', self.imu_callback, 10)
        self.mag_subscription = self.create_subscription(
            MagneticField, '/bno055/mag', self.mag_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.gps_subscription = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        self.bridge = CvBridge()

        # --- CSVログ機能用 ---
        self.log_directory = "sensor_logs"
        self.is_logging = False
        self.log_file = None
        self.csv_writer = None
        self.log_timer = None
        self.temp_log_path = None
        self.final_log_path = None

        # --- ハードウェア初期化 ---
        self.mcp_lock = threading.Lock()
        self.mcp = None

        self.h = None
        self.my_motor = None
        self.bme280 = None
        
        # モーターコントローラ (GritMotor / lgpio)
        try:
            self.h = lgpio.gpiochip_open(0)
            self.relay_pin = 17 # モーター電源リレー用
            lgpio.gpio_claim_output(self.h, self.relay_pin)
            self.my_motor = GritMotor(self.h)

            lgpio.gpio_write(self.h, self.relay_pin, 0)
            self.get_logger().info('Motor controller initialized successfully.')
        except Exception as e:  
            self.get_logger().error(f'Error initializing motor controller: {e}')
        
        # 環境センサ BME280
        try:
            self.bme280 = BME280(bus_number=1, i2c_address=0x76)
            self.get_logger().info('BME280 initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Error initializing BME280: {e}')

        # IOエキスパンダ MCP23017
        try:
            self.mcp = MCP23017(bus=1, address=0x20)
            self.mcp.setup(0, MCP23017.OUTPUT) # Buzzer
            self.mcp.output(0, 0) 
            self.mcp.setup(8, MCP23017.INPUT, pull_up=True) # Button
            self.get_logger().info('MCP23017 initialized successfully.')

            # 起動音
            self.get_logger().info('System Startup Beep...')
            self.mcp.output(0, 1)
            time.sleep(1.0)
            self.mcp.output(0, 0)

        except Exception as e:
            self.get_logger().error(f'MCP23017 Init Error: {e}')
            self.mcp = None

        # --- 外部プロセス管理（カメラ、IMU、GPSのROSノード） ---
        self.proc_cam = None 
        self.proc_imu = None
        self.proc_gps = None
        atexit.register(self.cleanup) # プログラム終了時のクリーンアップ登録
       
        self.command_queue = command_queue
        # コマンド処理用タイマー (0.1秒間隔)
        self.command_timer = self.create_timer(0.1, self.process_commands)
        # 環境センサ読み取りタイマー (1.0秒間隔)
        self.env_sensor_timer = self.create_timer(1.0, self.update_env_sensors)
        self.get_logger().info('ROS Subscriber Node has been started.')

    # --- コールバック関数群 ---
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

    def gps_callback(self, msg):
        global latest_gps_msg
        with gps_lock:
            latest_gps_msg = msg

    def update_env_sensors(self):
        """BME280から定期的にデータを取得"""
        if self.bme280:
            try:
                data = self.bme280.read_data()
                if data:
                    global latest_bme_data
                    with bme_lock:
                        latest_bme_data = data
            except Exception as e:
                self.get_logger().warn(f"Failed to read BME280: {e}")

    def process_commands(self):
        """
        コマンドキューから命令を取り出し、モーター制御やセンサー起動を実行する。
        WebSocketやRobotControllerからキューに追加される。
        """
        if not self.my_motor:
            return
        try:
            while not self.command_queue.empty():
                command_data = self.command_queue.get_nowait()
                command = command_data.get("command")
                left_speed = int(command_data.get("left", 0)) 
                right_speed = int(command_data.get("right", 0)) 

                if command == "move":
                    # 速度が0でない場合、リレーをONにしてモーター電源を供給
                    if left_speed != 0 or right_speed != 0:
                        lgpio.gpio_write(self.h, self.relay_pin, 1)
                    else:
                        lgpio.gpio_write(self.h, self.relay_pin, 0)
                    self.my_motor.move(left_speed, right_speed)

                elif command == "sensor":
                    # センサープロセスのON/OFF
                    sensor_type = command_data.get("sensor_type")
                    bin_val = int(command_data.get("bin"))
                    self.sensor_ctl(sensor_type, bin_val)

                elif command == "log":
                    # ログ記録の開始/停止
                    bin_val = int(command_data.get("msg"))
                    self.log_data_csv(bin_val)
                
                elif command == "io":
                    # GPIO制御
                    if self.mcp:
                        c_type = command_data.get("type")
                        pin = int(command_data.get("pin", 0))
                        with self.mcp_lock:
                            try:
                                if c_type == "setup":
                                    mode_str = command_data.get("mode", "out")
                                    mode = MCP23017.INPUT if mode_str == "in" else MCP23017.OUTPUT
                                    pull_up = True if mode == MCP23017.INPUT else False
                                    self.mcp.setup(pin, mode, pull_up=pull_up)
                                elif c_type == "write":
                                    val = int(command_data.get("val", 0))
                                    self.mcp.output(pin, 1 if val else 0)
                            except Exception as e:
                                self.get_logger().error(f"IO Command Error: {e}")

                else:
                    # 安全停止
                    self.my_motor.move(0, 0)
                    lgpio.gpio_write(self.h, self.relay_pin, 0)

        except queue.Empty:
            pass
        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
    
    def sensor_ctl(self, sensor_type, bin_val):
        """
        ROSノード（ドライバ）をサブプロセスとして起動・停止する。
        bin_val: 1=Start, 0=Stop
        """
        if sensor_type == "cam":
            proc_attr = "proc_cam"
            command = ["ros2", "run", "camera_ros", "camera_node", "--ros-args", "-p", "format:=YUYV"]
            log_prefix = "Camera"
            kill_pattern = "camera_ros" 

        elif sensor_type == "imu":
            proc_attr = "proc_imu"
            command = ["ros2", "launch", "bno055", "bno055.launch.py"]
            log_prefix = "IMU"
            kill_pattern = "bno055"

        elif sensor_type == "gps":
            proc_attr = "proc_gps"
            command = ["ros2", "run", "gpsd_driver", "gpsd_client_node"]
            log_prefix = "GPS"
            kill_pattern = "gpsd_client_node"

        else:
            self.get_logger().error(f"Unknown sensor type specified: {sensor_type}")
            return

        current_proc = getattr(self, proc_attr)

        if bin_val == 1:
            # 起動処理
            if current_proc and current_proc.poll() is None:
                self.get_logger().warn(f"{log_prefix} process is already running (managed).")
                return

            self.get_logger().info(f"Ensuring no zombie {log_prefix} processes exist...")
            force_kill_os_process(kill_pattern)
            time.sleep(0.5)

            try:
                self.get_logger().info(f"Starting {log_prefix}...")
                new_proc = subprocess.Popen(
                    command, 
                    start_new_session=True, # プロセスグループを分離
                    stdout=subprocess.DEVNULL, 
                    stderr=subprocess.DEVNULL
                )
                setattr(self, proc_attr, new_proc)
                self.get_logger().info(f"{log_prefix} started. PID: {new_proc.pid}")
            except Exception as e:
                self.get_logger().error(f"Failed to start {log_prefix}: {e}")

        else:
            # 停止処理
            self.get_logger().info(f"Stopping {log_prefix}...")
            if current_proc:
                self._stop_subprocess(current_proc, log_prefix)
                setattr(self, proc_attr, None)
            
            # 念のため強制killも実行
            force_kill_os_process(kill_pattern)
            self.get_logger().info(f"{log_prefix} stopped.")

    def _stop_subprocess(self, proc, name):
        """サブプロセスへSIGINT -> SIGKILLを送信して停止させる"""
        if proc.poll() is not None: return 
        try:
            pgid = os.getpgid(proc.pid)
            self.get_logger().info(f"Sending SIGINT to {name} (PGID: {pgid})...")
            os.killpg(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.get_logger().warn(f"{name} did not stop. Sending SIGKILL...")
                os.killpg(pgid, signal.SIGKILL)
                proc.wait(timeout=1)
        except ProcessLookupError:
            pass 
        except Exception as e:
            self.get_logger().error(f"Error stopping {name}: {e}")

    def log_data_csv(self, bin_command):
        """CSVログ記録の開始と終了処理"""
        if bin_command == 1 and not self.is_logging:
            try:
                os.makedirs(self.log_directory, exist_ok=True)
                self.is_logging = True
                base_filename = datetime.now().strftime("%Y%m%d_%H%M%S")
                # 書き込み中は _temp を付け、完了時にリネームする
                self.temp_log_path = os.path.join(self.log_directory, f"{base_filename}_temp.csv")
                self.final_log_path = os.path.join(self.log_directory, f"{base_filename}.csv")
                
                self.get_logger().info(f"Starting new log. Writing to temporary file: {self.temp_log_path}")
                self.log_file = open(self.temp_log_path, 'w', newline='')
                self.csv_writer = csv.writer(self.log_file)

                # CSVヘッダー
                header = [
                    "timestamp_sec", "timestamp_nanosec", 
                    "orient_x", "orient_y", "orient_z", "orient_w",
                    "ang_vel_x", "ang_vel_y", "ang_vel_z", 
                    "lin_accel_x", "lin_accel_y", "lin_accel_z",
                    "mag_x", "mag_y", "mag_z",
                    "gps_status", "latitude", "longitude", "altitude",
                    "temperature_celsius", "pressure_hpa", "humidity_percent",
                    "wifi_ssid", "wifi_signal_strength",
                    "cpu_temperature"
                ]
                self.csv_writer.writerow(header)
                self.log_timer = self.create_timer(0.1, self._write_log_callback)

            except (IOError, OSError) as e:
                self.get_logger().error(f"Failed to start logging: {e}")
                self.is_logging = False 

        elif bin_command == 0 and self.is_logging:
            self.is_logging = False
            if self.log_timer:
                self.log_timer.cancel()
                self.log_timer = None
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.csv_writer = None
            try:
                if self.temp_log_path and os.path.exists(self.temp_log_path):
                    os.rename(self.temp_log_path, self.final_log_path)
                    self.get_logger().info(f"Log file finalized: {self.final_log_path}")
            except (IOError, OSError) as e:
                self.get_logger().error(f"Failed to rename log file: {e}")
            self.temp_log_path = None
            self.final_log_path = None
        
    def _write_log_callback(self):
        """定期的にセンサーデータをCSVに書き込む"""
        if not self.is_logging or not self.csv_writer: return
        with imu_lock:
            imu_msg = latest_imu_msg
            mag_msg = latest_mag_msg
        with gps_lock:
            gps_msg = latest_gps_msg
        with system_info_lock:
            system_info = latest_system_info.copy()
        
        bme_data = None
        with bme_lock:
            if latest_bme_data: bme_data = latest_bme_data.copy()

        if not imu_msg: return
        
        # データの構築
        row = [
            imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec, 
            imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w,
            imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, 
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
        ]
        row.extend([mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z] if mag_msg else [None, None, None])
        if gps_msg:
            row.extend([gps_msg.status.status, gps_msg.latitude, gps_msg.longitude, gps_msg.altitude])
        else:
            row.extend([None, None, None, None])
        
        if bme_data:
            row.append(bme_data.get('temperature'))
            row.append(bme_data.get('pressure'))
            row.append(bme_data.get('humidity'))
        else:
            row.extend([None, None, None])

        row.append(system_info.get('wifi_ssid'))
        row.append(system_info.get('wifi_strength'))
        row.append(system_info.get('cpu_temp')) 
        self.csv_writer.writerow(row)

    def cleanup(self):
        """終了時のリソース解放"""
        self.get_logger().info("Cleaning up resources...")
        if self.is_logging: self.log_data_csv(0)
        
        # BME280終了
        if self.bme280:
            try: self.bme280.close()
            except Exception: pass
            self.bme280 = None

        # MCP23017終了
        if self.mcp:
            try:
                with self.mcp_lock:
                    self.mcp.output(0, 0)
                    self.mcp.cleanup()
            except Exception: pass
            self.mcp = None

        # サブプロセス停止
        if self.proc_cam: self._stop_subprocess(self.proc_cam, "Camera")
        if self.proc_imu: self._stop_subprocess(self.proc_imu, "IMU")
        if self.proc_gps: self._stop_subprocess(self.proc_gps, "GPS")

        # 念のため名前でkill
        force_kill_os_process("camera_node")
        force_kill_os_process("bno055")
        force_kill_os_process("gpsd_client")

        # モーター/GPIO終了
        if self.h:
            if self.my_motor:
                self.my_motor.move(0, 0)
                self.my_motor.cleanup()
            try:
                lgpio.gpio_write(self.h, self.relay_pin, 0)
                lgpio.gpiochip_close(self.h)
            except: pass
            self.h = None
            self.my_motor = None
            
        self.get_logger().info("Cleanup finished.")

# ==========================================================
# データ変換ヘルパー
# ==========================================================
def imu_to_dict(imu_msg: Imu):
    if not imu_msg: return None
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

def gps_to_dict(gps_msg: NavSatFix):
    if not gps_msg: return None
    return {
        'header': {
            'stamp': {'sec': gps_msg.header.stamp.sec, 'nanosec': gps_msg.header.stamp.nanosec},
            'frame_id': gps_msg.header.frame_id
        },
        'status': {
            'status': gps_msg.status.status,
            'service': gps_msg.status.service
        },
        'latitude': gps_msg.latitude,
        'longitude': gps_msg.longitude,
        'altitude': gps_msg.altitude,
        'position_covariance': list(gps_msg.position_covariance),
        'position_covariance_type': gps_msg.position_covariance_type
    }

def bme280_to_dict(bme_data: dict):
    if not bme_data: return None
    return {
        'temperature_celsius': bme_data.get('temperature'),
        'pressure_hpa': bme_data.get('pressure'),
        'humidity_percent': bme_data.get('humidity')
    }

# ==========================================================
# WebSocketクライアント
# 基地局サーバーとの通信を担当
# ==========================================================
class RobotWebsocketClient:
    def __init__(self, ros_node, script_manager, robot_id="robot03", server_uri="ws://<基地局PCのIPアドレス>:8000/ws/robot/"):
        self.ros_node = ros_node
        self.script_manager = script_manager
        self.uri = f"{server_uri}{robot_id}"
        self.bridge = CvBridge()
        self.ssid, self.strength = get_wifi_info()

        global CURRENT_ROBOT_ID
        CURRENT_ROBOT_ID = robot_id

        # OLEDにIDと接続情報を表示
        show_status_display()
        text_lines = []
        text_lines.append(f"id :  {robot_id}")
        text_lines.append("")
        if self.ssid and self.strength:
            text_lines.append(f"SSID :  {self.ssid}")
        else:
            text_lines.append("Wi-Fi Not Connected")
        oled.display_text(text_lines, start_x=5, start_y=5, line_spacing=12)

    async def run(self):
        """WebSocket接続を開始し、送受信タスクを並行実行"""
        async with websockets.connect(self.uri) as websocket:
            print(f"Connected to server: {self.uri}")
            listen_task = asyncio.create_task(self.listen_for_commands(websocket))
            send_task = asyncio.create_task(self.send_sensor_data(websocket))
            await asyncio.gather(listen_task, send_task)

    async def listen_for_commands(self, websocket):
        """サーバーからのJSONコマンドを受信して処理"""
        async for message in websocket:
            try:
                command_data = json.loads(message)
                command = command_data.get("command")
                print(f"Received command: {command_data}")
                if command == "set_robot_id":
                    new_id = command_data.get("new_id")
                    if new_id:
                        print(f"!!! ID Change Requested: {CURRENT_ROBOT_ID} -> {new_id} !!!")
                        
                        # 1. 設定ファイルに保存
                        if ConfigManager.update_robot_id(new_id):
                            # 2. ユーザーに通知 (OLED & ログ)
                            oled.clear()
                            oled.display_text(["", "ID CHANGED!", f"-> {new_id}", "Rebooting..."], start_x=0, start_y=0)
                            
                            # サーバーに成功レスポンスを返す（切断前に）
                            await websocket.send(json.dumps({
                                "type": "info",
                                "message": f"ID changed to {new_id}. Robot is restarting..."
                            }))
                            
                            await asyncio.sleep(2) # メッセージ送信とOLED表示の待機
                            
                            # 3. プログラムの再起動
                            print("Restarting application...")
                            # 現在のPythonインタプリタで、現在のスクリプトを引数付きで再実行する
                            os.execv(sys.executable, ['python3'] + sys.argv)
                        else:
                            await websocket.send(json.dumps({"type": "error", "message": "Failed to save config."}))
                elif command == "check_i2c_devices":
                    # スキャン実行
                    devices = scan_i2c_bus(bus_num=1)
                    
                    # 結果をフロントに返信
                    response = {
                        "type": "i2c_device_list",
                        "devices": devices
                    }
                    await websocket.send(json.dumps(response))
                    print(f"Sent I2C device list: {devices}")    
                elif command == "save_code":
                    code = command_data.get("code")
                    if code: self.script_manager.save_code(code)
                elif command == "start_program":
                    self.script_manager.start_program()
                elif command == "stop_program":
                    self.script_manager.stop_program()
                elif command == "list_log_files":
                    await self.handle_list_log_files(websocket)
                elif command == "get_log_file":
                    filename = command_data.get("filename")
                    if filename: await self.handle_get_log_file(websocket, filename)
                    else:
                        await websocket.send(json.dumps({"type": "error", "message": "'filename' is required."}))
                else:
                    # その他のコマンドはROSノードのコマンドキューへ
                    self.ros_node.command_queue.put(command_data)

            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
            except Exception as e:
                print(f"Error processing command: {e}")

    async def send_sensor_data(self, websocket):
        """定期的にセンサー情報と画像をサーバーへ送信"""
        while True:
            # データのスナップショット取得
            with imu_lock:
                imu_msg = latest_imu_msg
                mag_msg = latest_mag_msg
            with image_lock:
                image_msg = latest_image_msg
            with gps_lock:
                gps_msg = latest_gps_msg
            bme_read = None
            with bme_lock:
                if latest_bme_data: bme_read = latest_bme_data.copy()
            
            payload = {"type": "sensor_data", "data": {}}
            
            # --- 各種センサーデータの格納 ---
            if imu_msg:
                payload["data"]["imu"] = imu_to_dict(imu_msg)
                try:
                    q = imu_msg.orientation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    payload["data"]["compass"] = (90.0 - math.degrees(yaw)) % 360.0
                except Exception:
                    payload["data"]["compass"] = None
            
            if mag_msg:
                payload["data"]["mag"] = {
                    'header': {'stamp': {'sec': mag_msg.header.stamp.sec, 'nanosec': mag_msg.header.stamp.nanosec}, 'frame_id': mag_msg.header.frame_id},
                    'magnetic_field': {'x': mag_msg.magnetic_field.x, 'y': mag_msg.magnetic_field.y, 'z': mag_msg.magnetic_field.z}
                }
            if gps_msg:
                payload["data"]["gps"] = gps_to_dict(gps_msg)
            
            if bme_read: payload["data"]["bme280"] = bme280_to_dict(bme_read)
            else: payload["data"]["bme280"] = None
                
            ssid, strength = get_wifi_info()
            cpu_temp = get_cpu_temperature() 
            with system_info_lock:
                if ssid and strength:
                    latest_system_info['wifi_ssid'] = ssid
                    latest_system_info['wifi_strength'] = strength
                latest_system_info['cpu_temp'] = cpu_temp 

            payload["data"]["wifi"] = {"ssid": ssid, "signal_strength": strength}
            payload["data"]["cpu_temperature"] = cpu_temp

            # --- 画像送信ロジック (優先度処理) ---
            image_data_b64 = None
            
            # 1. ユーザープログラムからのデバッグ画像があるか確認 (0.5秒以内に更新されたもの)
            use_debug_image = False
            debug_img = None
            
            global latest_debug_image, last_debug_update_time
            with debug_image_lock:
                if latest_debug_image is not None:
                    if time.time() - last_debug_update_time < 0.5:
                        debug_img = latest_debug_image.copy()
                        use_debug_image = True
            
            if use_debug_image and debug_img is not None:
                try:
                    # デバッグ画像は既に正立しているのでそのままエンコード
                    ret, frame = cv2.imencode('.jpg', debug_img)
                    if ret:
                        image_data_b64 = base64.b64encode(frame).decode('utf-8')
                except Exception as e:
                    print(f"Debug image encoding error: {e}")

            # 2. デバッグ画像がない場合は、通常のROSカメラ画像を使用
            if image_data_b64 is None and image_msg:
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                    # 生データは逆さなので反転して正立にする
                    cv_image_flipped = cv2.flip(cv_image, -1)
                    ret, frame = cv2.imencode('.jpg', cv_image_flipped)
                    if ret:
                        image_data_b64 = base64.b64encode(frame).decode('utf-8')
                except Exception as e:
                    print(f"Raw image processing error: {e}")

            if image_data_b64:
                payload["data"]["image"] = image_data_b64

            if payload["data"]:
                await websocket.send(json.dumps(payload))

            await asyncio.sleep(0.033)  # 約30fps

    async def handle_list_log_files(self, websocket):
        """ログディレクトリ内のファイル一覧を送信"""
        try:
            log_dir = self.ros_node.log_directory
            if not os.path.isdir(log_dir):
                await websocket.send(json.dumps({"type": "error", "message": f"Log directory not found: {log_dir}"}))
                return
            files = [f for f in os.listdir(log_dir) if f.endswith('.csv') and not f.endswith('_temp.csv')]
            await websocket.send(json.dumps({"type": "log_file_list", "files": files}))
        except Exception as e:
            await websocket.send(json.dumps({"type": "error", "message": str(e)}))

    async def handle_get_log_file(self, websocket, filename):
        """指定されたログファイルの内容を送信"""
        try:
            if ".." in filename or filename.startswith("/"): raise ValueError("Invalid filename specified.")
            log_dir = self.ros_node.log_directory
            file_path = os.path.join(log_dir, filename)
            if os.path.exists(file_path):
                with open(file_path, 'r', encoding='utf-8') as f:
                    file_content = f.read()
                await websocket.send(json.dumps({"type": "log_file_content", "filename": filename, "data": file_content}))
            else:
                await websocket.send(json.dumps({"type": "error", "message": "File not found.", "filename": filename}))
        except Exception as e:
            await websocket.send(json.dumps({"type": "error", "message": str(e), "filename": filename}))

def run_ros_spin(node):
    """ROSイベントループを実行するスレッド関数"""
    print("ROS spin thread started.")
    try: rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException: pass
    finally:
        node.cleanup() 
        node.destroy_node()
        print("ROS Node destroyed.")


if __name__ == "__main__":
    rclpy.init()

    command_queue = queue.Queue()
    ros_node = RosSubscriberNode(command_queue)
    script_manager = ScriptManager(ros_node)

    # 物理ボタン監視スレッド
    def button_watcher():
        while True:
            script_manager.check_button()
            time.sleep(0.1)

    button_thread = threading.Thread(target=button_watcher, daemon=True)
    button_thread.start()
    
    # ROSスレッド
    ros_thread = threading.Thread(target=run_ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # WebSocketクライアント起動
    target_uri = f"ws://{SERVER_IP}:{SERVER_PORT}/ws/robot/"

    client = RobotWebsocketClient(
        ros_node=ros_node, 
        script_manager=script_manager,
        robot_id=CURRENT_ROBOT_ID, # Configから読み込んだID
        server_uri=target_uri
    )

    try:
        print("Starting WebSocket client...")
        asyncio.run(client.run())

    except KeyboardInterrupt:
        print("Application stopped by user (Ctrl+C).")
        
    finally:
        print("Shutting down rclpy...")
        rclpy.shutdown()
        ros_thread.join(timeout=2)
        try:
            if 'client' in locals() and hasattr(client, 'bme280') and client.bme280:
                client.bme280.close()
        except Exception: pass
        oled.clear()
        print("Application has exited.")