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
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack, RTCConfiguration, RTCIceServer
from av import VideoFrame

import queue
from concurrent.futures import ThreadPoolExecutor
import json
import websockets

import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse

from cv_bridge import CvBridge
import numpy as np
import math
import csv
from datetime import datetime

import lgpio
# OLED はなくてもロボットとして動作できるので、ライブラリ (luma.oled/Pillow) が無くても起動を止めない
try:
    from src.oled import OLEDDisplay
    _oled_import_error = None
except Exception as e:
    OLEDDisplay = None
    _oled_import_error = e
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
import hashlib
import traceback

import logging
# logging.basicConfig(level=logging.DEBUG)
# logging.getLogger("aioice").setLevel(logging.DEBUG)

# ==========================================================
# グローバル変数と排他制御 (Locks)
# ==========================================================
# ROSスレッドとメインスレッド/WebSocketスレッド間でデータを共有するための変数
latest_imu_msg = None
latest_image_msg = None
latest_image_time = 0.0  # 最後にカメラ画像を受信した時刻 (time.monotonic)。スナップショットの鮮度判定に使う
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
oled_lock = threading.Lock()

# asyncio の補正: 閉じ終わった UDP トランスポートへの sendto を無視する
# WebRTC の接続を閉じた後も、aioice (aiortc 内部) の STUN/TURN 再送タイマーが閉じたソケットに送ることがある。
# asyncio はその場合 _sock も _loop も None になっているため、
#   AttributeError: 'NoneType' object has no attribute 'call_exception_handler'
# を送信元に投げてしまい、映像の再接続 (WebRTC 交渉) が失敗する (Python 3.10 / 3.12.0 で再現確認済み)。
# 閉じかけのトランスポートへの送信は asyncio 自身も捨てる (ただし警告ログを大量に出す) ので、
# 閉じ終わった場合も含めて黙って捨てる。
def _patch_asyncio_datagram_sendto():
    import asyncio.selector_events as _se
    original = _se._SelectorDatagramTransport.sendto
    if getattr(original, "_gritcat_patched", False):
        return

    def sendto(self, data, addr=None):
        if self._sock is None or self._conn_lost:
            return  # 閉じている/閉じかけ
        return original(self, data, addr)

    sendto._gritcat_patched = True
    _se._SelectorDatagramTransport.sendto = sendto

_patch_asyncio_datagram_sendto()

# C-APIの設定 (非同期例外注入時のクラッシュ・セグフォ防止)
ctypes.pythonapi.PyThreadState_SetAsyncExc.argtypes = [ctypes.c_long, ctypes.py_object]
ctypes.pythonapi.PyThreadState_SetAsyncExc.restype = ctypes.c_int

command_queue = queue.Queue()
shutdown_event = threading.Event()

# --- デッドマン (手動操作の通信途絶時の自動停止) ---
# Web UI は操作ボタンを押している間 0.2秒ごとに move を送り直す。
# 手動操作の move がこの秒数届かなければモーターを止める (ユーザープログラムの move は対象外)
DEADMAN_TIMEOUT_SEC = 1.0
MANUAL_MOVE_SOURCES = ("remote", "local_dashboard")

SENSOR_DISPLAY_NAMES = {"cam": "カメラ", "imu": "IMU", "gps": "GPS"}
# センサーノード (ros2 run/launch) の出力の保存先。起動のたびに上書き
SENSOR_LOG_DIR = "/tmp/gritcat-sensors"
SENSOR_STARTUP_CHECK_SEC = 3.0  # 起動後この秒数たっても動いていれば「起動成功」とみなす

def sensor_log_path(sensor_type):
    return os.path.join(SENSOR_LOG_DIR, f"{sensor_type}.log")

def read_log_tail(path, lines=3, max_chars=300):
    """ログの末尾 (空行を除く数行) を1行にまとめて返す。読めなければ空文字"""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            tail = [l.strip() for l in f.read().splitlines() if l.strip()][-lines:]
        return " / ".join(tail)[-max_chars:]
    except OSError:
        return ""

# OLEDディスプレイの初期化
class NullOLED:
    """OLED が使えないときの代わり。表示命令はすべて何もしない (呼び出し側の分岐を不要にする)"""
    def display_text(self, *args, **kwargs):
        pass

    def clear(self):
        pass

def init_oled():
    """OLED を初期化する。失敗してもシステムは止めず、(oled, エラー内容) を返す"""
    if OLEDDisplay is None:
        return NullOLED(), f"OLED ライブラリを読み込めません: {_oled_import_error}"
    try:
        return OLEDDisplay(font_size=15), None
    except Exception as e:
        return NullOLED(), f"OLED を初期化できません: {e}"

oled, OLED_INIT_ERROR = init_oled()
if OLED_INIT_ERROR:
    print(f"[WARN] {OLED_INIT_ERROR} — OLED 表示なしで起動を続けます。")

# 設定をロード
config = ConfigManager.load_config()
CURRENT_ROBOT_ID = config.get("robot_id", "robot_unknown")
SERVER_IP = config.get("server_ip", "192.168.11.14") # IPもConfig管理する場合
SERVER_PORT = config.get("server_port", 8000)
SERVER_URL = config.get("server_url")

if SERVER_URL:
    # URLが指定されている場合
    # 末尾が / で終わっていない場合は補完する
    if not SERVER_URL.endswith('/'):
        SERVER_URL += '/'
    TARGET_URI = SERVER_URL
else:
    # URLがない場合は、従来通り IP と Port から構築する
    SERVER_IP = config.get("server_ip", "192.168.11.14")
    SERVER_PORT = config.get("server_port", 8000)
    TARGET_URI = f"ws://{SERVER_IP}:{SERVER_PORT}/ws/robot/"

print(f"Target WebSocket URI: {TARGET_URI}{CURRENT_ROBOT_ID}")

# ==========================================================
# カメラ設定 (撮影と配信を分けて管理。ローカルダッシュボードから変更し config.json の "camera" に保存)
#   撮影: カメラノードの解像度/FPS。画像処理ブロックやスナップショットはこの解像度
#   配信: WebRTC で送る映像の横幅/FPS (高さは撮影のアスペクト比から自動計算)
# ==========================================================
CAMERA_DEFAULTS = {
    "capture_width": 320, "capture_height": 240, "capture_fps": 5,
    "stream_width": 320, "stream_fps": 5,
}
CAPTURE_RESOLUTIONS = [(320, 240), (640, 480), (800, 600), (1280, 720)]
STREAM_WIDTHS = [320, 480, 640]
CAMERA_FPS_CHOICES = [5, 10, 15]
SNAPSHOT_JPEG_QUALITY = 90
SNAPSHOT_MAX_IMAGE_AGE_SEC = 2.0  # これより古い画像しかなければカメラ停止中とみなす

def validate_camera_settings(data):
    """カメラ設定を検証して正規化した dict を返す。不正なら ValueError"""
    s = {key: int(data.get(key, default)) for key, default in CAMERA_DEFAULTS.items()}
    if (s["capture_width"], s["capture_height"]) not in CAPTURE_RESOLUTIONS:
        raise ValueError(f"未対応の撮影解像度です: {s['capture_width']}x{s['capture_height']}")
    if s["capture_fps"] not in CAMERA_FPS_CHOICES or s["stream_fps"] not in CAMERA_FPS_CHOICES:
        raise ValueError("FPS は 5 / 10 / 15 から選んでください")
    if s["stream_width"] not in STREAM_WIDTHS:
        raise ValueError(f"未対応の配信横幅です: {s['stream_width']}")
    if s["stream_width"] > s["capture_width"]:
        raise ValueError("配信の横幅は撮影の横幅以下にしてください")
    if s["stream_fps"] > s["capture_fps"]:
        raise ValueError("配信FPSは撮影FPS以下にしてください (超えた分は同じ画像の再送になるだけです)")
    return s

try:
    camera_settings = validate_camera_settings(config.get("camera", {}))
except (ValueError, TypeError) as e:
    print(f"Invalid camera settings in config.json ({e}). Using defaults.")
    camera_settings = dict(CAMERA_DEFAULTS)
camera_settings_lock = threading.Lock()

def get_camera_settings():
    with camera_settings_lock:
        return dict(camera_settings)

# ==========================================================
# ヘルパー関数
# ==========================================================
def build_target_uri(conf):
    """設定辞書からWebSocket URIを構築するヘルパー"""
    uri = conf.get("server_url", "")
    if uri:
        if not uri.endswith('/'): uri += '/'
        return f"{uri}{conf.get('robot_id')}"
    else:
        ip = conf.get("server_ip", "192.168.11.14")
        port = conf.get("server_port", 8000)
        return f"ws://{ip}:{port}/ws/robot/{conf.get('robot_id')}"

# --- Wi-Fi情報のキャッシュ ---
# get_wifi_info() は iwconfig をサブプロセスで起動する重い処理(最大1.5秒ブロック)のため、
# 専用スレッドで定期取得し、他の箇所はキャッシュを参照する
_wifi_cache = ("N/A", "N/A")
_wifi_cache_lock = threading.Lock()

def _wifi_monitor_loop(interval=2.0):
    global _wifi_cache
    while True:
        info = get_wifi_info()
        with _wifi_cache_lock:
            _wifi_cache = info
        time.sleep(interval)

def get_cached_wifi_info():
    """最新の (ssid, signal_level) をブロックせずに返す"""
    with _wifi_cache_lock:
        return _wifi_cache

# --- ロボット → Web UI への通知 (イベントバス) ---
# 各スレッド(ユーザープログラム・センサー制御など)から emit_robot_event() で積み、
# RobotWebsocketClient.forward_events() がサーバーへ送る。接続が切れている間は上限まで溜めて再接続後に送る
USER_PROGRAM_FILENAME = "user_program.py"  # エラー行の特定に使うコンパイル時のファイル名
EVENT_MESSAGE_MAX_LEN = 500
robot_event_queue = queue.Queue(maxsize=500)

def emit_robot_event(event_type, message, level="info", **extra):
    """
    Web UI のコンソールに表示するイベントを積む (どのスレッドからでも呼べる)。
    event_type: "robot_event" | "program_status" | "program_output"
    level: "info" | "success" | "warning" | "error"
    """
    event = {"type": event_type, "level": level, "message": str(message)[:EVENT_MESSAGE_MAX_LEN],
             "time": time.time(), **extra}
    try:
        robot_event_queue.put_nowait(event)
    except queue.Full:
        pass  # 溢れた場合は捨てる (通信途絶中に古いイベントで埋まるのを防ぐ)

class ProgramOutputForwarder:
    """
    ユーザープログラムの print() の代わり。標準出力に出しつつ Web UI へ転送する。
    ループ内の print で通信が溢れないよう、1秒あたりの転送行数を制限し、超過分は件数だけ通知する。
    """
    MAX_LINES_PER_SEC = 20

    def __init__(self):
        self._window_start = time.monotonic()
        self._sent_in_window = 0
        self._dropped = 0

    def __call__(self, *args, sep=' ', end='\n', **kwargs):
        print(*args, sep=sep, end=end, **kwargs)
        now = time.monotonic()
        if now - self._window_start >= 1.0:
            self.flush_dropped()
            self._window_start = now
            self._sent_in_window = 0
        if self._sent_in_window < self.MAX_LINES_PER_SEC:
            self._sent_in_window += 1
            emit_robot_event("program_output", sep.join(str(a) for a in args))
        else:
            self._dropped += 1

    def flush_dropped(self):
        if self._dropped:
            emit_robot_event("program_output", f"(出力が多いため {self._dropped} 行を省略しました)", level="warning")
            self._dropped = 0

# info表示から通常画面へ戻すタイマー (新しいinfo表示が来たら前のタイマーは取り消す)
_oled_revert_timer = None

def _schedule_oled_revert(delay):
    global _oled_revert_timer
    if _oled_revert_timer:
        _oled_revert_timer.cancel()
    _oled_revert_timer = threading.Timer(delay, update_oled, kwargs={"mode": "connection"})
    _oled_revert_timer.daemon = True
    _oled_revert_timer.start()

def update_oled(text_lines=None, mode=None, clear=False, start_x=0, start_y=0, line_spacing=12):
    """
    OLEDディスプレイの表示を一元管理する関数。
    """
    try:
        with oled_lock:
            if clear:
                oled.clear()
                if text_lines is None and mode is None:
                    return

            if mode:
                display_lines = []
                display_lines.append(f"ID : {CURRENT_ROBOT_ID}")
                if mode == "connection":
                    ssid, strength = get_cached_wifi_info()
                    if ssid:
                        display_lines.append(f"Wi-Fi: {ssid[:12]}") 
                        display_lines.append(f"Signal: {strength} dBm")
                    else:
                        display_lines.append("Wi-Fi: Disconnected")

                elif mode == "custom":
                    if text_lines is not None:
                        if isinstance(text_lines, list):
                            display_lines.extend(text_lines)
                        else:
                            display_lines.append(text_lines)
                elif mode == "error":
                    display_lines.append("Error occurred!")
                    if text_lines is not None:
                        if isinstance(text_lines, list):
                            display_lines.extend(text_lines)
                        else:
                            display_lines.append(text_lines)
                elif mode == "info":
                    if text_lines is not None:
                        if isinstance(text_lines, list):
                            display_lines.extend(text_lines)
                        else:
                            display_lines.append(text_lines)
                    else:
                        display_lines.append("debug info")
                    
                    oled.display_text(display_lines, start_x=0, start_y=0, line_spacing=12)
                    # 情報表示は3秒後に通常画面へ戻す (呼び出し元をブロックしないようタイマーで)
                    _schedule_oled_revert(3.0)
                    return

                oled.display_text(display_lines, start_x=0, start_y=0, line_spacing=12)
            elif text_lines is not None:
                oled.display_text(text_lines, start_x=start_x, start_y=start_y, line_spacing=line_spacing)

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

def stream_frame_size(settings):
    """配信フレームの (幅, 高さ)。高さは撮影のアスペクト比から計算し、エンコーダ向けに偶数へ丸める"""
    w = settings["stream_width"]
    h = round(w * settings["capture_height"] / settings["capture_width"] / 2) * 2
    return w, h

class ROSCameraTrack(VideoStreamTrack):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.last_frame_time = 0.0

    async def recv(self):
        """
        WebRTCが次のフレームを要求したときに呼ばれるメソッド。
        ここで画像処理の結果を選択して返す。
        配信FPS・サイズは毎フレーム設定を参照するので、ローカルダッシュボードでの変更が即座に反映される。
        """
        settings = get_camera_settings()
        frame_duration = 1.0 / settings["stream_fps"]
        current_time = time.time()
        elapsed = current_time - self.last_frame_time
        wait = frame_duration - elapsed
        if wait > 0:
            await asyncio.sleep(wait)
        
        self.last_frame_time = time.time()
        
        pts, time_base = await self.next_timestamp()
        
        cv_img = None
        
        # --- 優先順位 1: ユーザープログラムのデバッグ画像 (latest_debug_image) ---
        global latest_debug_image, last_debug_update_time
        with debug_image_lock:
            # 0.5秒以内に更新されていれば採用
            if latest_debug_image is not None and (time.time() - last_debug_update_time < 0.5):
                cv_img = latest_debug_image.copy()

        # --- 優先順位 2: 生のカメラ画像 (latest_image_msg) ---
        if cv_img is None:
            with image_lock:
                if latest_image_msg is not None:
                    try:
                        # ROSメッセージ -> OpenCV形式
                        cv_img = self.ros_node.bridge.imgmsg_to_cv2(latest_image_msg, 'bgr8')
                        # 生データは逆さなので反転して正立にする
                        cv_img = cv2.flip(cv_img, -1)
                    except Exception as e:
                        print(f"WebRTC Image conversion error: {e}")

        # --- 優先順位 3: 画像が全くない場合は黒画面 ---
        if cv_img is None:
            cv_img = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(cv_img, "Waiting for Camera...", (180, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

         
        # OpenCV(BGR) -> PyAV VideoFrame に変換して送信
        # aiortc/av は BGR24 形式を受け入れ可能
        cv_img = cv2.resize(cv_img, stream_frame_size(settings))
        # --- デバッグ処理：画像上にロボットIDと現在時刻を描画 ---
        cv2.putText(
            cv_img, 
            f"ROBOT ID: {CURRENT_ROBOT_ID} | {datetime.now().strftime('%H:%M:%S.%f')[:-3]}", 
            (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.7, 
            (0, 0, 255), # 赤色文字
            2
        )
        # ----------------------------------------------------
        frame = VideoFrame.from_ndarray(cv_img, format="bgr24")
        frame.pts = pts
        frame.time_base = time_base
        
        return frame

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

        elif sensor_type in ('bme', 'bme280'):  # ブロックエディタは 'bme280' を出力する
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
            ssid, rssi = get_cached_wifi_info()
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
        """ブザー制御（MCP23017のPin 7に接続されている想定）"""
        self.digital_write(7, 1 if enable else 0)

    def print_display(self, message):
        """OLEDディスプレイにメッセージを表示"""
        self._check_stop()
        try:
            lines = []
            if isinstance(message, list):
                lines = [str(x) for x in message]
            else:
                lines = str(message).split('\n')
            update_oled(text_lines=lines, start_x=0, start_y=0, line_spacing=15)
        except Exception as e:
            print(f"[OLED Error]: {e}")

    def print(self, text):
        """コンソールへのログ出力（プレフィックス付き）。プログラム実行中は Web UI にも転送される"""
        output = getattr(self, "_output", None) or print
        output(f"[Robot]: {text}")

    # ----------------------------------------------------------
    #  画像処理 (OpenCV)
    # ----------------------------------------------------------
    def get_image(self):
        """
        【新規】カメラから最新の画像を取得し、OpenCV形式(BGR)で直接返す。
        内部状態(_cv_image)は変更しない（ステートレス用）。
        """
        self._check_stop()
        global latest_image_msg
        
        with image_lock:
            if latest_image_msg is None:
                self.print("Warning: No camera image received yet.")
                return None
            try:
                # ROSメッセージ -> OpenCV画像変換
                cv_img = self.ros_node.bridge.imgmsg_to_cv2(latest_image_msg, desired_encoding='bgr8')
                # カメラ取り付け向きに合わせて正立にする
                cv_img = cv2.flip(cv_img, -1)
                return cv_img
            except Exception as e:
                self.print(f"Image get error: {e}")
                return None
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

        # ブロックの割り算等で float が渡されるため整数化 (スライスは int 必須)
        x, y, w, h = int(x), int(y), int(w), int(h)
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
            kernel_size = max(1, int(kernel_size))
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
        size = int(size)

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
        w, h, thickness = int(w), int(h), int(thickness)
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
        """
        受信したコードを構文チェックしてからファイルに保存する。
        戻り値: (ok, message)。構文エラーの場合は保存せず、直前の正常なプログラムを残す
        """
        normalized_code = code_str.replace('\r\n', '\n').replace('\r', '\n')
        try:
            compile(normalized_code, USER_PROGRAM_FILENAME, "exec")
        except SyntaxError as e:
            msg = f"構文エラーのため書き込みませんでした ({e.lineno}行目: {e.msg})"
            print(msg)
            return False, msg
        try:
            with open(self.script_path, "w", encoding="utf-8") as f:
                f.write(normalized_code)
            lines = normalized_code.count('\n') + (0 if normalized_code.endswith('\n') else 1)
            print(f"Code saved to {self.script_path}")
            return True, f"プログラムを書き込みました ({lines}行)"
        except Exception as e:
            print(f"Save error: {e}")
            return False, f"書き込みに失敗しました: {e}"

    def is_running(self):
        return bool(self.execution_thread and self.execution_thread.is_alive())

    def start_program(self):
        """
        ユーザープログラムを別スレッドで実行開始。
        戻り値: (ok, message)。実行結果 (終了/エラー) は program_status イベントで別途通知する
        """
        if self.is_running():
            print("Program is already running.")
            return False, "プログラムはすでに実行中です"

        if not os.path.exists(self.script_path):
            print("No program file found.")
            return False, "ロボットにプログラムがありません。先に書き込んでください"

        print(">>> Starting User Program >>>")
        self.is_stopping = False
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
        return True, "プログラムの実行を開始しました"

    def stop_program(self):
        """
        実行中のプログラムを強制停止。
        戻り値: (ok, message)。停止完了は program_status イベントで通知する
        """
        result = (True, "実行中のプログラムはありません (モーターを停止しました)")
        if self.is_running():
            result = (True, "プログラムの停止を要求しました")
            if getattr(self, 'is_stopping', False):
                self.ros_node.command_queue.put({"command": "move", "left": 0, "right": 0})
                return True, "停止処理中です"
            self.is_stopping = True
            
            print(">>> Requesting User Script to stop gracefully... >>>")
            self.stop_event.set()
            
            # asyncioのイベントループをブロックしないよう、別スレッドで終了を監視
            def watchdog(target_thread):
                # 1秒待機し、それでも終了しなければKeyboardInterruptを注入
                target_thread.join(timeout=1.0)
                if target_thread.is_alive():
                    print(">>> Script is stubborn. Injecting KeyboardInterrupt... >>>")
                    raise_keyboard_interrupt(target_thread)
                    
                    target_thread.join(timeout=5.0)
                    if target_thread.is_alive():
                        print("Warning: Script is STILL stubborn. Blocking in C?")
                        emit_robot_event("program_status", "プログラムを停止できません (処理が応答しません)。ロボットの再起動が必要な場合があります",
                                         level="error", state="stuck")
                
                self.is_stopping = False
            
            threading.Thread(target=watchdog, args=(self.execution_thread,), daemon=True).start()

        # 安全のためモーター停止コマンドを送信
        self.ros_node.command_queue.put({"command": "move", "left": 0, "right": 0})
        # stop_event.clear() はここでは行わず、次回のstart_program()で行う
        return result

    def _run_script_thread(self, code_str):
        """実際にユーザースクリプトを実行するスレッド本体"""
        robot = RobotController(self.ros_node, self.stop_event)
        program_print = ProgramOutputForwarder()
        robot._output = program_print  # robot.print() の警告も同じ経路で転送

        # ユーザープログラム内で利用可能な変数・モジュールを定義
        local_scope = {
            "robot": robot,
            "time": time,
            "np": np,
            "print": program_print,  # 出力を Web UI のコンソールにも転送
            "math": __import__("math")
        }
        try:
            print(">>> User Script Started >>>")
            emit_robot_event("program_status", "プログラムを開始しました", state="started")
            update_oled(text_lines=["", "   Program", "   Running...", ""], start_x=5, start_y=5, line_spacing=15)
            # 文字列として渡されたPythonコードを実行
            # globals と locals に同じ辞書を渡す。分けると def 内から robot が見えず、
            # Blockly が出力する global 変数もトップレベルと別物になってしまう
            # ファイル名付きでコンパイルし、エラー時にユーザープログラムの行番号を特定できるようにする
            exec(compile(code_str, USER_PROGRAM_FILENAME, "exec"), local_scope)
            print("<<< User Script Finished Normally <<<")
            program_print.flush_dropped()
            emit_robot_event("program_status", "プログラムが終了しました", state="finished")

        except (KeyboardInterrupt, InterruptedError):
            # InterruptedError は robot.* の呼び出し中に停止要求を検知したときに送出される
            print("\n!!! User Script Interrupted by System (Stop Command) !!!")
            program_print.flush_dropped()
            emit_robot_event("program_status", "プログラムを停止しました", level="warning", state="stopped")
            update_oled(text_lines=["", "   STOPPED", "", ""], start_x=5, start_y=5)
            time.sleep(1.0)

        except SyntaxError as e:
            print(f"!!! Syntax Error in User Script: line {e.lineno} !!!\n{e}")
            emit_robot_event("program_status", f"構文エラー ({e.lineno}行目): {e.msg}",
                             level="error", state="error", line=e.lineno)

        except Exception as e:
            print(f"!!! Runtime Error in User Script: {e} !!!")
            import traceback
            traceback.print_exc()
            # ユーザープログラム内で最後に実行していた行を探す
            user_frames = [f for f in traceback.extract_tb(e.__traceback__) if f.filename == USER_PROGRAM_FILENAME]
            line = user_frames[-1].lineno if user_frames else None
            where = f" ({line}行目)" if line else ""
            program_print.flush_dropped()
            emit_robot_event("program_status", f"実行時エラー{where}: {type(e).__name__}: {e}",
                             level="error", state="error", line=line)

        finally:
            print("--- Safety Cleanup: Stopping Motors ---")
            robot.stop()
            update_oled(mode="connection")

    def check_button(self):
        """
        MCP23017のPin 15を監視
        - 0.1秒〜3秒未満: プログラムの開始/停止
        - 3秒〜8秒未満: システムの再起動 (プロセス初期化)
        - 8秒以上: システムのシャットダウン (ラズパイ電源OFF)
        """
        if not self.ros_node.mcp: return
        
        try:
            # 0が押下状態 (Pull-up)
            is_pressed = False
            with self.ros_node.mcp_lock:
                if self.ros_node.mcp.input(15) == 0:
                    is_pressed = True

            if is_pressed:
                press_start = time.time()
                shutdown_triggered = False
                
                # フィードバック用のフラグ
                flag_3s = False
                
                # ボタンが押されている間ループ
                while True:
                    time.sleep(0.1)
                    elapsed = time.time() - press_start
                    
                    # 押下継続確認
                    with self.ros_node.mcp_lock:
                        if self.ros_node.mcp.input(15) != 0:
                            break # ボタンが離された
                    
                    # --- 3秒経過: 「再起動」のスタンバイ状態 ---
                    if elapsed >= 3.0 and not flag_3s:
                        flag_3s = True
                        self._beep(0.1) # 「ピッ」と短く鳴らす
                        update_oled(text_lines=["", " Release to", " RESTART", ""], clear=True, start_x=5, start_y=5)
                    
                    # --- 8秒経過: 「シャットダウン」発動 ---
                    if elapsed >= 8.0:
                        shutdown_triggered = True
                        self._beep(1.0) # 「ピーー」と長く鳴らす
                        self.system_shutdown()
                        break
                
                # --- ボタンが離された時の判定 ---
                if not shutdown_triggered:
                    duration = time.time() - press_start
                    
                    if duration >= 3.0:
                        # 3秒〜8秒の間で離された -> システム再起動
                        self.system_restart()
                    elif duration > 0.1:
                        # 0.1秒〜3秒の間で離された -> プログラム開始/停止 (チャタリング防止)
                        if self.execution_thread and self.execution_thread.is_alive():
                            print("Button: Stop Program")
                            self.stop_program()
                        else:
                            print("Button: Start Program")
                            self.start_program()
                        
                        # ボタンが完全に離されるのを待つ
                        self._wait_for_release(15)
                        # ステータス表示に戻す
                        update_oled(mode="connection")

        except Exception as e:
            print(f"Button Check Error: {e}")

    def _beep(self, duration):
        """ブザーを指定秒数鳴らすヘルパーメソッド"""
        if not self.ros_node.mcp: return
        try:
            with self.ros_node.mcp_lock:
                self.ros_node.mcp.output(7, 1)
            time.sleep(duration)
            with self.ros_node.mcp_lock:
                self.ros_node.mcp.output(7, 0)
        except:
            pass

    def system_restart(self):
        """システム（Pythonプロセス）を安全に再起動する"""
        print("!!! System Restart Sequence Started !!!")
        
        # 1. 実行中のユーザープログラムを停止
        self.stop_program()
        
        # 2. ディスプレイに通知
        update_oled(text_lines=["", "  SYSTEM", "  RESTARTING...", ""], clear=True, start_x=0, start_y=0)
        
        # 3. モーターの安全停止
        self.ros_node.command_queue.put({"command": "move", "left": 0, "right": 0})
        
        # 少し待機してリソースの解放を待つ
        time.sleep(1.5)
        
        # 4. OSレベルで現在のPythonスクリプトを再実行
        print("Restarting application via os.execv...")
        os.execv(sys.executable, ['python3'] + sys.argv)

            
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
    def system_shutdown(self):
        """システムを安全に停止し、電源を切る"""
        print("!!! System Shutdown Sequence Started !!!")
        
        # 1. 実行中のユーザープログラムを停止
        self.stop_program()
        
        # 2. ディスプレイに通知
        update_oled(text_lines=["", "  SHUTTING DOWN", "  PLEASE WAIT...", ""], clear=True, start_x=0, start_y=0)
        
        # 3. モーターの安全停止（念押し）
        self.ros_node.command_queue.put({"command": "move", "left": 0, "right": 0})
        
        # 4. 少し待機してOSにシャットダウン命令を出す
        time.sleep(2.0)
        os.system("sudo shutdown -h now")


# ==========================================================
# ROS 2 ノード
# センサー購読、ハードウェア制御、外部プロセス管理
# ==========================================================
class RosSubscriberNode(Node):
    def __init__(self, command_queue):
        super().__init__('ros_fastapi_subscriber')
        
        # --- センサーのサブスクライバ設定 ---
        self.imu_subscription = self.create_subscription(
            Imu, f'/{CURRENT_ROBOT_ID}/bno055/imu', self.imu_callback, 10)
        self.mag_subscription = self.create_subscription(
            MagneticField, f'/{CURRENT_ROBOT_ID}/bno055/mag', self.mag_callback, 10)
        self.image_subscription = self.create_subscription(
            Image, f'/{CURRENT_ROBOT_ID}/camera/image_raw', self.image_callback, 10)
        self.gps_subscription = self.create_subscription(
            NavSatFix, f'/{CURRENT_ROBOT_ID}/gps/fix', self.gps_callback, 10)
        
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
        self.motor_init_error = None
        self.bme280 = None
        
        # モーターコントローラ (GritMotor / lgpio)
        try:
            self.h = lgpio.gpiochip_open(0)
            #self.relay_pin = 17 # モーター電源リレー用
            self.stby_pin=22
            #lgpio.gpio_claim_output(self.h, self.relay_pin)
            lgpio.gpio_claim_output(self.h, self.stby_pin)
            self.my_motor = GritMotor(self.h)

            #lgpio.gpio_write(self.h, self.relay_pin, 0)
            lgpio.gpio_write(self.h, self.stby_pin, 1)
            self.get_logger().info('Motor controller initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Error initializing motor controller: {e}')
            # センサー等は動くのに移動だけできない状態になるので、原因を Web UI に知らせる
            # (よくある原因: GPIO busy = 別の main.py がすでに動いていてピンを使用中)
            self.my_motor = None
            self.motor_init_error = str(e)
            emit_robot_event("robot_event", f"モーターを初期化できませんでした: {e}。移動の指示は実行されません。", level="error")
        self._last_motor_unavailable_notice = 0.0

        # 環境センサ BME280
        try:
            self.bme280 = BME280(bus_number=1, i2c_address=0x76)
            self.get_logger().info('BME280 initialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Error initializing BME280: {e}')

        # IOエキスパンダ MCP23017
        try:
            self.mcp = MCP23017(bus=1, address=0x20)
            self.mcp.setup(7, MCP23017.OUTPUT) # Buzzer
            self.mcp.output(0, 0) 
            self.mcp.setup(15, MCP23017.INPUT, pull_up=True) # Button
            self.get_logger().info('MCP23017 initialized successfully.')

            # 起動音
            self.get_logger().info('System Startup Beep...')
            self.mcp.output(7, 1)
            time.sleep(1.0)
            self.mcp.output(7, 0)

        except Exception as e:
            self.get_logger().error(f'MCP23017 Init Error: {e}')
            self.mcp = None

        # --- 外部プロセス管理（カメラ、IMU、GPSのROSノード） ---
        self.proc_cam = None 
        self.proc_imu = None
        self.proc_gps = None
        # センサープロセスの起動/停止は数秒かかるため、ROSスレッドを止めないよう専用スレッドで直列実行する
        self.sensor_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="sensor_ctl")
        atexit.register(self.cleanup) # プログラム終了時のクリーンアップ登録

        self.command_queue = command_queue
        # 手動操作で走行中のとき、停止すべき時刻 (time.monotonic)。None なら監視しない
        self.manual_move_deadline = None
        # コマンド処理用タイマー (0.02秒間隔: 操作の遅延を抑える)
        self.command_timer = self.create_timer(0.02, self.process_commands)
        # 環境センサ読み取りタイマー (1.0秒間隔)
        self.env_sensor_timer = self.create_timer(1.0, self.update_env_sensors)
        self.get_logger().info('ROS Subscriber Node has been started.')

    # --- コールバック関数群 ---
    def imu_callback(self, msg):
        global latest_imu_msg
        with imu_lock:
            latest_imu_msg = msg

    def image_callback(self, msg):
        global latest_image_msg, latest_image_time
        with image_lock:
            latest_image_msg = msg
            latest_image_time = time.monotonic()
    
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
        # 溜まっているコマンドをまとめて取り出す
        commands = []
        while True:
            try:
                commands.append(self.command_queue.get_nowait())
            except queue.Empty:
                break

        # 移動指示は最新の1件だけ反映すればよい (古い指示を順に実行すると操作が遅れる)
        last_move_index = max(
            (i for i, c in enumerate(commands) if c.get("command") == "move"), default=-1)

        for i, command_data in enumerate(commands):
            try:
                command = command_data.get("command")

                if command == "move":
                    if i != last_move_index:
                        continue
                    if not self.my_motor:
                        self._notify_motor_unavailable(command_data)
                        continue
                    left_speed = int(command_data.get("left", 0))
                    right_speed = int(command_data.get("right", 0))
                    # デッドマン: 手動操作で走行中のみ期限を設ける (停止指示やプログラムからの指示で解除)
                    if command_data.get("source") in MANUAL_MOVE_SOURCES and (left_speed or right_speed):
                        self.manual_move_deadline = time.monotonic() + DEADMAN_TIMEOUT_SEC
                    else:
                        self.manual_move_deadline = None
                    # 速度が0でない場合、リレーをONにしてモーター電源を供給
                    #if left_speed != 0 or right_speed != 0:
                        #lgpio.gpio_write(self.h, self.relay_pin, 1)
                    #else:
                        #lgpio.gpio_write(self.h, self.relay_pin, 0)
                    self.my_motor.move(left_speed, right_speed)

                elif command == "sensor":
                    # センサープロセスのON/OFF (時間がかかるので専用スレッドへ)
                    sensor_type = command_data.get("sensor_type")
                    bin_val = int(command_data.get("bin"))
                    self.sensor_executor.submit(self._sensor_ctl_with_feedback, sensor_type, bin_val)

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
                    if self.my_motor:
                        self.my_motor.move(0, 0)
                    #lgpio.gpio_write(self.h, self.relay_pin, 0)

            except Exception as e:
                self.get_logger().error(f"Error processing command {command_data}: {e}")

        self._check_deadman()

    def _notify_motor_unavailable(self, command_data):
        """モーター未初期化で移動指示を捨てたことを知らせる (操作中は指示が連続するので5秒に1回まで)"""
        left, right = command_data.get("left", 0), command_data.get("right", 0)
        if not (left or right):
            return  # 停止指示は捨てても問題ない
        now = time.monotonic()
        if now - self._last_motor_unavailable_notice < 5.0:
            return
        self._last_motor_unavailable_notice = now
        reason = f" (初期化エラー: {self.motor_init_error})" if self.motor_init_error else ""
        self.get_logger().warn(f"Move command ignored: motor controller is not available{reason}")
        emit_robot_event("robot_event", f"モーターが使えないため移動できません{reason}。ロボットの再起動や、別のプログラムが GPIO を使用していないか確認してください。", level="error")

    def _check_deadman(self):
        """手動操作の move が DEADMAN_TIMEOUT_SEC 途絶えたらモーターを止める"""
        if self.manual_move_deadline is None or time.monotonic() < self.manual_move_deadline:
            return
        self.manual_move_deadline = None
        self.get_logger().warn(
            f"Deadman: no manual move command for {DEADMAN_TIMEOUT_SEC}s. Stopping motors.")
        if self.my_motor:
            self.my_motor.move(0, 0)
        emit_robot_event("robot_event", f"操作指示が{DEADMAN_TIMEOUT_SEC}秒途絶えたためモーターを停止しました", level="warning")

    def _sensor_ctl_with_feedback(self, sensor_type, bin_val):
        """sensor_ctl を実行し、結果をOLEDと Web UI に通知する (sensor_executor 上で実行)"""
        name = SENSOR_DISPLAY_NAMES.get(sensor_type, str(sensor_type))
        try:
            self.sensor_ctl(sensor_type, bin_val)
            update_oled(mode="info", text_lines=[f"{str(sensor_type).capitalize()}:", f"{'Started' if bin_val else 'Stopped'}"])
            if not bin_val:
                emit_robot_event("robot_event", f"{name}を停止しました")
                return
            proc = getattr(self, f"proc_{sensor_type}", None)
            # 起動直後は生きていても、設定やデバイスの問題で数秒以内に終了することがあるので少し待って確かめる
            if proc and proc.poll() is None:
                time.sleep(SENSOR_STARTUP_CHECK_SEC)
            if proc and proc.poll() is None:
                emit_robot_event("robot_event", f"{name}を起動しました", level="success")
            else:
                code = proc.returncode if proc else None
                tail = read_log_tail(sensor_log_path(sensor_type))
                detail = f" (終了コード {code})" if code is not None else ""
                emit_robot_event("robot_event",
                                 f"{name}の起動に失敗しました{detail}。" + (f"出力: {tail}" if tail else "")
                                 + f" 詳細: {sensor_log_path(sensor_type)}", level="error")
        except Exception as e:
            self.get_logger().error(f"Sensor control error ({sensor_type}): {e}")
            emit_robot_event("robot_event", f"{name}の操作でエラーが発生しました: {e}", level="error")

    def restart_camera_if_running(self):
        """
        撮影設定を反映するため、カメラが起動中なら再起動する (sensor_executor 上で実行)。
        カメラノードの解像度/FPSは起動時パラメータでしか変えられないため再起動が必要。
        """
        def _restart():
            if self.proc_cam and self.proc_cam.poll() is None:
                self.get_logger().info("Restarting camera to apply new capture settings...")
                self.sensor_ctl("cam", 0)
                self.sensor_ctl("cam", 1)
                update_oled(mode="info", text_lines=["Camera:", "Settings applied"])
        self.sensor_executor.submit(_restart)

    def sensor_ctl(self, sensor_type, bin_val):
        """
        ROSノード（ドライバ）をサブプロセスとして起動・停止する。
        bin_val: 1=Start, 0=Stop
        """
        env = os.environ.copy()
        if sensor_type == "cam":
            proc_attr = "proc_cam"
            cam = get_camera_settings()  # 撮影解像度/FPSはローカルダッシュボードで設定
            command = [
                "ros2", "run", "camera_ros", "camera_node",
                "--ros-args",
                "-p", "format:=YUYV",
                "-p", f"width:={cam['capture_width']}",
                "-p", f"height:={cam['capture_height']}",
                "-p", f"frame_rate:={float(cam['capture_fps'])}",
                "-r", f"__ns:=/{CURRENT_ROBOT_ID}"
            ]
            log_prefix = "Camera"
            kill_pattern = "camera_ros" 

        elif sensor_type == "imu":
            proc_attr = "proc_imu"
            # GritCat 用の起動ファイル (gritcat_utils) から bno055 ノードを namespace 付きで起動する。
            # 外部パッケージ bno055 側の launch/パラメータは書き換えずに使う
            command = ["ros2", "launch", "gritcat_utils", "bno055.launch.py",
                       f"namespace:={CURRENT_ROBOT_ID}"
                       ]
            log_prefix = "IMU"
            kill_pattern = "bno055"

        elif sensor_type == "gps":
            proc_attr = "proc_gps"
            command = ["ros2", "run", "gpsd_driver", "gpsd_client_node",
                       "--ros-args",
                       "-r", 
                       f"/gps/fix:=/{CURRENT_ROBOT_ID}/gps/fix"]
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
                # 出力は捨てずにログファイルへ (起動に失敗したとき原因を Web UI に表示するため)
                os.makedirs(SENSOR_LOG_DIR, exist_ok=True)
                with open(sensor_log_path(sensor_type), "w") as log_file:
                    new_proc = subprocess.Popen(
                        command,
                        start_new_session=True, # プロセスグループを分離
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                        env=env
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
                    "timestamp_sec", "timestamp_curr", 
                    "orient_x", "orient_y", "orient_z", "orient_w",
                    "compass",
                    "ang_vel_x", "ang_vel_y", "ang_vel_z", 
                    "lin_accel_x", "lin_accel_y", "lin_accel_z",
                    "mag_x", "mag_y", "mag_z",
                    "gps_status", "latitude", "longitude", "altitude", "gps_h_err", "gps_v_err",
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
        
    def _write_log_callback(self,hz=10):
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

        row = [time.time(), datetime.now().isoformat()]

        if imu_msg:
            q = imu_msg.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            compass = (90.0 - math.degrees(yaw)) % 360.0
            row.extend([
                imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w,
                compass,
                imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z, 
                imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z
            ])
        else:
            row.extend([None] * 11)

        row.extend([mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z] if mag_msg else [None, None, None])
        if gps_msg:
            is_no_fix = (gps_msg.status.status == -1) or math.isnan(gps_msg.latitude)
            lat = 403.0 if is_no_fix else gps_msg.latitude
            lon = 403.0 if is_no_fix else gps_msg.longitude
            alt = 403.0 if is_no_fix else gps_msg.altitude
            
            h_err = 403.0
            v_err = 403.0
            if not is_no_fix and len(gps_msg.position_covariance) == 9:
                cov_e = gps_msg.position_covariance[0]
                cov_n = gps_msg.position_covariance[4]
                cov_u = gps_msg.position_covariance[8]
                if cov_e >= 0 and cov_n >= 0:
                    h_err = math.sqrt(cov_e + cov_n)
                if cov_u >= 0:
                    v_err = math.sqrt(cov_u)
            
            row.extend([gps_msg.status.status, lat, lon, alt, h_err, v_err])
        else:
            row.extend([-1, 403.0, 403.0, 403.0, 403.0, 403.0])
        
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
        self.log_file.flush()
        
        

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

        # 未実行のセンサー起動/停止要求を破棄 (終了処理後にプロセスが立ち上がらないように)
        self.sensor_executor.shutdown(wait=False, cancel_futures=True)

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
                #lgpio.gpio_write(self.h, self.relay_pin, 0)
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
    if not gps_msg:
        return {
            'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'no_gps'},
            'status': {'status': -1, 'service': 0},
            'latitude': 403.0,
            'longitude': 403.0,
            'altitude': 403.0,
            'h_err': 403.0,
            'v_err': 403.0,
            'position_covariance': [0.0]*9,
            'position_covariance_type': 0
        }
    
    is_no_fix = (gps_msg.status.status == -1) or math.isnan(gps_msg.latitude)
    lat = 403.0 if is_no_fix else gps_msg.latitude
    lon = 403.0 if is_no_fix else gps_msg.longitude
    alt = 403.0 if is_no_fix else gps_msg.altitude

    h_err = 403.0
    v_err = 403.0
    if not is_no_fix and len(gps_msg.position_covariance) == 9:
        cov_e = gps_msg.position_covariance[0]
        cov_n = gps_msg.position_covariance[4]
        cov_u = gps_msg.position_covariance[8]
        if cov_e >= 0 and cov_n >= 0:
            h_err = math.sqrt(cov_e + cov_n)
        if cov_u >= 0:
            v_err = math.sqrt(cov_u)

    return {
        'header': {
            'stamp': {'sec': gps_msg.header.stamp.sec, 'nanosec': gps_msg.header.stamp.nanosec},
            'frame_id': gps_msg.header.frame_id
        },
        'status': {
            'status': gps_msg.status.status,
            'service': gps_msg.status.service
        },
        'latitude': lat,
        'longitude': lon,
        'altitude': alt,
        'h_err': h_err,
        'v_err': v_err,
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
# システム状態管理 (Mode & Config)
# ==========================================================
class SystemState:
    def __init__(self):
        self.mode = config.get("operation_mode", "local") # デフォルトはlocal
        self.lock = threading.Lock()

    def set_mode(self, new_mode):
        with self.lock:
            self.mode = new_mode
            # config.jsonにも保存
            current_conf = ConfigManager.load_config()
            current_conf["operation_mode"] = new_mode
            ConfigManager.save_config(current_conf)

    def get_mode(self):
        with self.lock:
            return self.mode

system_state = SystemState()

# ==========================================================
# FastAPI アプリケーション定義
# ==========================================================
app = FastAPI()

@app.get("/", response_class=HTMLResponse)
async def index():
    return DASHBOARD_HTML

@app.get("/api/status")
async def get_status():
    """センサーデータのサマリーを返す"""
    with imu_lock:
        imu = imu_to_dict(latest_imu_msg) if latest_imu_msg else None
    with bme_lock:
        bme = latest_bme_data.copy() if latest_bme_data else None
    
    wifi_ssid, wifi_rssi = get_cached_wifi_info()
    return {
        "mode": system_state.get_mode(),
        "robot_id": CURRENT_ROBOT_ID,
        "sensors": {
            "imu": imu,
            "bme": bme,
            "cpu_temp": get_cpu_temperature(),
            "wifi": {"ssid": wifi_ssid, "rssi": wifi_rssi}
        }
    }

@app.post("/api/move")
async def local_move(data: dict):
    """ローカルUIからの移動操作 (Localモード時のみ有効)"""
    if system_state.get_mode() != "local":
        return JSONResponse(content={"status": "denied", "reason": "System is in REMOTE mode"}, status_code=403)
    
    command_queue.put({
        "command": "move",
        "left": data.get("left", 0),
        "right": data.get("right", 0),
        "source": "local_dashboard"
    })
    return {"status": "ok"}

@app.post("/api/mode")
async def set_mode(data: dict):
    """動作モードの切り替え"""
    new_mode = data.get("mode")
    if new_mode in ["local", "remote"]:
        system_state.set_mode(new_mode)
        # モード切替時に安全のため停止
        command_queue.put({"command": "move", "left": 0, "right": 0})
        return {"mode": system_state.get_mode()}
    return JSONResponse(content={"error": "Invalid mode"}, status_code=400)

# --- API: 設定の取得 ---
@app.get("/api/config")
async def get_config_api():
    return ConfigManager.load_config()

# --- API: 設定の保存 ---
@app.post("/api/config")
async def save_config_api(data: dict):
    """config.jsonの書き換え"""
    new_conf = ConfigManager.load_config()
    
    # 既存のキー名に合わせて更新
    if "robot_id" in data: new_conf["robot_id"] = data["robot_id"]
    if "server_ip" in data: new_conf["server_ip"] = data["server_ip"]
    if "server_port" in data: new_conf["server_port"] = int(data["server_port"])
    if "server_url" in data: new_conf["server_url"] = data["server_url"]
    if "operation_mode" in data: new_conf["operation_mode"] = data["operation_mode"]
    
    if ConfigManager.save_config(new_conf):
        global TARGET_URI, CURRENT_ROBOT_ID
        CURRENT_ROBOT_ID = new_conf["robot_id"]
        TARGET_URI = build_target_uri(new_conf)
        
        # WebSocketクライアントのインスタンスがある場合は、そのURIも更新
        if 'client' in globals():
            client.uri = TARGET_URI
            print(f"Updated running client URI to: {client.uri}")
        return {"status": "success"}
    return JSONResponse(content={"error": "Failed to save config"}, status_code=500)

# --- API: カメラ設定 ---
@app.get("/api/camera")
async def get_camera_api():
    """現在のカメラ設定と、画面に出す選択肢を返す"""
    return {
        "settings": get_camera_settings(),
        "options": {
            "capture_resolutions": [list(r) for r in CAPTURE_RESOLUTIONS],
            "stream_widths": STREAM_WIDTHS,
            "fps": CAMERA_FPS_CHOICES,
        },
    }

@app.post("/api/camera")
async def save_camera_api(data: dict):
    """カメラ設定を検証・保存し、即座に反映する"""
    global camera_settings
    try:
        new_settings = validate_camera_settings(data)
    except (ValueError, TypeError) as e:
        return JSONResponse(content={"error": str(e)}, status_code=400)

    conf = ConfigManager.load_config()
    conf["camera"] = new_settings
    if not ConfigManager.save_config(conf):
        return JSONResponse(content={"error": "Failed to save config"}, status_code=500)

    with camera_settings_lock:
        old_settings = camera_settings
        camera_settings = new_settings

    # 配信設定は ROSCameraTrack が毎フレーム参照するので即反映。撮影設定はカメラ再起動が必要
    capture_keys = ("capture_width", "capture_height", "capture_fps")
    capture_changed = any(old_settings[k] != new_settings[k] for k in capture_keys)
    node = globals().get("ros_node")
    camera_restarted = False
    if capture_changed and node and node.proc_cam and node.proc_cam.poll() is None:
        node.restart_camera_if_running()
        camera_restarted = True
    return {"status": "success", "settings": new_settings, "camera_restarted": camera_restarted}

# --- API: Wi-Fi アクセスポイント設定 ---
# Netplan の変更は root 権限が必要なため、root 所有のヘルパーを sudo 経由で呼ぶ
# (インストール: sudo bash app/tools/install_wifi_helper.sh)。
# パスワードはここで WPA の PSK (64桁hex) に変換し、平文はヘルパーにも保存先にも渡さない。
WIFI_HELPER = "/usr/local/sbin/gritcat-wifi-helper"
WIFI_HELPER_MISSING = ("Wi-Fi 設定ヘルパーが未インストールです。ロボット上で "
                       "sudo bash app/tools/install_wifi_helper.sh を実行してください。")

def wpa_psk(ssid, passphrase):
    """WPA/WPA2-Personal の PSK を計算する (wpa_passphrase と同じ PBKDF2-SHA1, 4096回)"""
    return hashlib.pbkdf2_hmac("sha1", passphrase.encode("utf-8"), ssid.encode("utf-8"), 4096, 32).hex()

def run_wifi_helper(cmd, payload=None, timeout=40):
    """ヘルパーを実行して結果 dict を返す。入力は標準入力の JSON で渡す (ps に表示させない)"""
    if not os.path.exists(WIFI_HELPER):
        return {"ok": False, "error": WIFI_HELPER_MISSING}
    try:
        r = subprocess.run(["sudo", "-n", WIFI_HELPER, cmd],
                           input=json.dumps(payload) if payload is not None else "",
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "Wi-Fi 設定の処理がタイムアウトしました"}
    lines = r.stdout.strip().splitlines()
    try:
        return json.loads(lines[-1])
    except (IndexError, json.JSONDecodeError):
        if "password is required" in r.stderr or "not allowed" in r.stderr:
            return {"ok": False, "error": WIFI_HELPER_MISSING}
        return {"ok": False, "error": f"Wi-Fi 設定ヘルパーの実行に失敗しました: {r.stderr.strip()[:200]}"}

def wifi_response(result):
    return result if result.get("ok") else JSONResponse(content=result, status_code=400)

@app.get("/api/wifi")
async def get_wifi_api():
    """登録済みアクセスポイント一覧と、現在接続中の SSID"""
    result = await asyncio.to_thread(run_wifi_helper, "list")
    ssid, signal = get_cached_wifi_info()
    result["current"] = {"ssid": None if ssid == "N/A" else ssid, "signal_dbm": None if signal == "N/A" else signal}
    return wifi_response(result)

@app.get("/api/wifi/scan")
async def scan_wifi_api():
    return wifi_response(await asyncio.to_thread(run_wifi_helper, "scan"))

@app.post("/api/wifi")
async def add_wifi_api(data: dict):
    """アクセスポイントを追加 (同じ SSID は上書き)。WPA/WPA2-Personal のみ対応"""
    ssid = str(data.get("ssid", ""))  # 前後の空白も SSID の一部になりうるので削らない
    password = str(data.get("password", ""))
    if not ssid.strip():
        return JSONResponse(content={"ok": False, "error": "SSID を入力してください"}, status_code=400)
    if not (8 <= len(password) <= 63) or any(not (0x20 <= ord(c) <= 0x7e) for c in password):
        return JSONResponse(content={"ok": False, "error": "パスワードは半角英数記号 8〜63 文字で入力してください"},
                            status_code=400)
    return wifi_response(await asyncio.to_thread(run_wifi_helper, "add", {"ssid": ssid, "psk": wpa_psk(ssid, password)}))

@app.post("/api/wifi/remove")
async def remove_wifi_api(data: dict):
    """ローカルダッシュボードで追加したアクセスポイントを削除。接続中のものは confirm が必要"""
    ssid = str(data.get("ssid", ""))
    current, _ = get_cached_wifi_info()
    if ssid == current and not data.get("confirm"):
        return JSONResponse(content={"ok": False, "needs_confirm": True,
                                     "error": f"「{ssid}」は現在接続中です。削除すると接続が切れる可能性があります。"},
                            status_code=409)
    return wifi_response(await asyncio.to_thread(run_wifi_helper, "remove", {"ssid": ssid}))

@app.post("/api/restart")
async def restart_system():
    """システムを完全に再起動する"""
    def delayed_restart():
        time.sleep(1.0)
        print("Restarting process...")
        os.execv(sys.executable, ['python3'] + sys.argv)
    
    threading.Thread(target=delayed_restart).start()
    return {"status": "restarting"}
# ==========================================================
# ローカルダッシュボード HTML
# ==========================================================
DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Local Dashboard</title>
    <style>
        body { font-family: -apple-system, sans-serif; background: #f4f7f9; color: #333; margin: 0; padding: 20px; }
        .container { max-width: 800px; margin: auto; }
        .card { background: white; border-radius: 12px; padding: 20px; box-shadow: 0 4px 6px rgba(0,0,0,0.05); margin-bottom: 20px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .btn { padding: 12px; border: none; border-radius: 8px; cursor: pointer; font-weight: bold; transition: 0.2s; }
        .btn-move { background: #3b82f6; color: white; width: 100%; margin-top: 10px; }
        input { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 6px; margin-top: 5px; box-sizing: border-box; background: #fafafa; }
        label { font-size: 0.85em; color: #555; font-weight: bold; display: block; margin-top: 10px; }
        .hint { font-size: 0.75em; color: #888; margin-top: 4px; }
        .section-title { border-left: 4px solid #3b82f6; padding-left: 10px; margin: 20px 0 10px 0; font-size: 1.1em; }
        .sub-title { font-size: 0.95em; margin: 10px 0 0 0; color: #333; }
        .btn-sub { background: #e5e7eb; color: #333; white-space: nowrap; }
        .wifi-list { list-style: none; padding: 0; margin: 8px 0 0 0; }
        .wifi-list li { display: flex; align-items: center; gap: 8px; padding: 8px 10px; border: 1px solid #eee; border-radius: 6px; margin-bottom: 6px; }
        .wifi-list .ssid { flex: 1; font-weight: bold; word-break: break-all; }
        .tag { font-size: 0.72em; padding: 2px 8px; border-radius: 999px; background: #eef2ff; color: #3b82f6; }
        .tag-muted { background: #f3f4f6; color: #6b7280; }
        .btn-remove { padding: 6px 10px; background: transparent; color: #dc2626; border: 1px solid #dc2626; }
        select { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 6px; margin-top: 5px; box-sizing: border-box; background: #fafafa; }
        @media (max-width: 600px) { .grid { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <h1>ロボット管理ダッシュボード</h1>
        
        <div class="grid">
            <!-- モード設定 -->
            <div class="card">
                <h2 class="section-title">動作モード</h2>
                <select id="modeSelect" style="width:100%; padding:12px; border-radius:6px;" onchange="updateMode()">
                    <option value="local">LOCAL (ローカル操作)</option>
                    <option value="remote">REMOTE (基地局サーバー操作)</option>
                </select>
                <p id="modeDesc" class="hint">現在: ---</p>
            </div>

            <!-- ステータス -->
            <div class="card">
                <h2 class="section-title">接続状態</h2>
                <div id="statusList" style="font-size:0.9em;">
                    Loading status...
                </div>
            </div>
            
        </div>

        <!-- システム設定 -->
        <div class="card">
            <h2 class="section-title">システム設定 (config.json)</h2>
            
            <label>Robot ID</label>
            <input type="text" id="conf_id" placeholder="例: robot01">

            <div style="display: flex; gap: 15px; margin-top: 10px;">
                <div style="flex: 3;">
                    <label>サーバー IPアドレス</label>
                    <input type="text" id="conf_ip" placeholder="192.168.x.x">
                </div>
                <div style="flex: 1;">
                    <label>ポート</label>
                    <input type="number" id="conf_port" placeholder="8000">
                </div>
            </div>
            <p class="hint">※URIが空の時にこのIP/ポートが使用されます。</p>

            <label style="margin-top: 20px;">サーバー固定 URI (WebSocket)</label>
            <input type="text" id="conf_url" placeholder="ws://example.com/ws/robot/">
            <p class="hint">※ここに入力がある場合、IP設定より優先されます。</p>

            <button class="btn btn-move" onclick="saveConfig()">設定を保存して反映</button>
        </div>

        <!-- Wi-Fi 設定 -->
        <div class="card">
            <h2 class="section-title">Wi-Fi アクセスポイント</h2>
            <p id="wifi_current" class="hint">現在の接続: ---</p>

            <h3 class="sub-title">登録済み</h3>
            <ul id="wifi_list" class="wifi-list"><li class="hint">読み込み中…</li></ul>

            <h3 class="sub-title" style="margin-top:16px;">追加</h3>
            <div style="display:flex; gap:10px; align-items:flex-end;">
                <div style="flex:1;">
                    <label for="wifi_ssid">SSID</label>
                    <input type="text" id="wifi_ssid" list="wifi_scan_results" autocomplete="off" placeholder="ネットワーク名">
                    <datalist id="wifi_scan_results"></datalist>
                </div>
                <button class="btn btn-sub" onclick="scanWifi()" id="wifi_scan_btn">周辺をスキャン</button>
            </div>
            <label for="wifi_password">パスワード</label>
            <input type="password" id="wifi_password" autocomplete="new-password" placeholder="8〜63文字 (WPA/WPA2-Personal)">
            <button class="btn btn-move" onclick="addWifi()">追加して反映</button>
            <p id="wifi_message" class="hint"></p>
            <p class="hint">
                ※追加したアクセスポイントは既存のネットワーク設定に<b>追加</b>されます (既存の設定は変更しません)。<br>
                ※反映時に Wi-Fi が数秒切れることがあります。反映後 90 秒以内に Wi-Fi につながらなかった場合は、自動で直前の設定に戻ります。
            </p>
        </div>

        <!-- カメラ設定 -->
        <div class="card">
            <h2 class="section-title">カメラ設定</h2>
            <div class="grid">
                <div>
                    <h3 class="sub-title">撮影</h3>
                    <label for="cam_capture_res">解像度</label>
                    <select id="cam_capture_res" onchange="updateCameraOptionStates()"></select>
                    <label for="cam_capture_fps">FPS</label>
                    <select id="cam_capture_fps" onchange="updateCameraOptionStates()"></select>
                    <p class="hint">画像処理ブロックとスナップショットはこの解像度です。変更するとカメラが再起動し、映像が数秒途切れます。</p>
                </div>
                <div>
                    <h3 class="sub-title">映像配信 (WebRTC)</h3>
                    <label for="cam_stream_width">横幅</label>
                    <select id="cam_stream_width"></select>
                    <label for="cam_stream_fps">FPS</label>
                    <select id="cam_stream_fps"></select>
                    <p class="hint">高さは撮影のアスペクト比に合わせて自動で決まります。配信は撮影の解像度・FPS以下に制限されます。即時反映されます。</p>
                </div>
            </div>
            <button class="btn btn-move" onclick="saveCameraConfig()">カメラ設定を保存して反映</button>
            <p id="cam_message" class="hint"></p>
        </div>

        <div class="card">
            <h2 class="section-title">システム操作</h2>
            <p class="hint">設定変更後は再起動を行うことで、全てのセンサーと接続設定が完全にリセット・反映されます。</p>
            <button class="btn btn-restart" style="background:#6b7280; color:white; width:100%;" onclick="restartSystem()">システムを再起動 (Reflesh)</button>
        </div>
    </div>

    <script>
        // --- 1. ページ読み込み時に1回だけ実行する処理 (Configの読み込み) ---
        async function loadInitialConfig() {
            try {
                const resConf = await fetch('/api/config');
                const conf = await resConf.json();
                
                // 入力欄に値をセット
                document.getElementById('conf_id').value = conf.robot_id || "";
                document.getElementById('conf_ip').value = conf.server_ip || "";
                document.getElementById('conf_port').value = conf.server_port || 8000;
                document.getElementById('conf_url').value = conf.server_url || "";
                document.getElementById('modeSelect').value = conf.operation_mode || "local";
                
                console.log("Config loaded once.");
            } catch(e) {
                console.error("Failed to load initial config", e);
            }
        }

        // --- 2. 定期的に実行する処理 (センサー状態の更新) ---
        async function updateStatus() {
            try {
                const resStat = await fetch('/api/status');
                const stat = await resStat.json();
                
                // モード表示（バッジとテキストのみ。セレクトボックスは勝手に書き換えない）
                const badge = document.getElementById('modeBadge');
                if (badge) {
                    badge.innerText = stat.mode.toUpperCase();
                    badge.className = 'mode-badge mode-' + stat.mode;
                }
                document.getElementById('modeDesc').innerText = "現在稼働モード: " + stat.mode.toUpperCase();

                // センサー情報のみを更新
                document.getElementById('statusList').innerHTML = `
                    ID: <b>${stat.robot_id}</b><br>
                    CPU温度: <b>${stat.sensors.cpu_temp} ℃</b><br>
                    Wi-Fi: <b>${stat.sensors.wifi.ssid}</b> (${stat.sensors.wifi.rssi} dBm)
                `;
            } catch(e) {
                console.warn("Status update failed (server might be busy)");
            }
        }

        // --- 3. ボタン操作などのイベント処理 ---
        
        async function updateMode() {
            const m = document.getElementById('modeSelect').value;
            await fetch('/api/mode', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({mode: m})
            });
            // 即座にステータス表示に反映
            updateStatus();
        }

        async function saveConfig() {
            const data = {
                robot_id: document.getElementById('conf_id').value,
                server_ip: document.getElementById('conf_ip').value,
                server_port: document.getElementById('conf_port').value,
                server_url: document.getElementById('conf_url').value,
                operation_mode: document.getElementById('modeSelect').value
            };
            
            const res = await fetch('/api/config', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(data)
            });

            if(res.ok) {
                alert("設定を保存しました。反映にはプログラムの再起動を推奨します。");
            } else {
                alert("保存に失敗しました。");
            }
        }
        // --- Wi-Fi 設定 ---
        // SSID は周辺の誰でも自由に名付けられる値なので、HTML として埋め込まず textContent で表示する
        function wifiItem(ssid, tags, removable) {
            const li = document.createElement('li');
            const name = document.createElement('span');
            name.className = 'ssid';
            name.textContent = ssid;
            li.appendChild(name);
            for (const [text, muted] of tags) {
                const t = document.createElement('span');
                t.className = muted ? 'tag tag-muted' : 'tag';
                t.textContent = text;
                li.appendChild(t);
            }
            if (removable) {
                const b = document.createElement('button');
                b.className = 'btn btn-remove';
                b.textContent = '削除';
                b.onclick = () => removeWifi(ssid);
                li.appendChild(b);
            }
            return li;
        }

        async function loadWifi() {
            const list = document.getElementById('wifi_list');
            try {
                const res = await fetch('/api/wifi');
                const body = await res.json();
                const cur = body.current || {};
                document.getElementById('wifi_current').textContent = cur.ssid
                    ? `現在の接続: ${cur.ssid} (${cur.signal_dbm} dBm)` : '現在の接続: なし';
                list.innerHTML = '';
                if (!body.ok) {
                    list.appendChild(Object.assign(document.createElement('li'), { className: 'hint', textContent: body.error }));
                    return;
                }
                const rows = [
                    ...(body.others || []).map(s => [s, [['システム設定', true]], false]),
                    ...(body.managed || []).map(s => [s, [['ダッシュボードで追加', false]], true]),
                ];
                if (!rows.length) list.appendChild(Object.assign(document.createElement('li'), { className: 'hint', textContent: '登録されていません' }));
                for (const [ssid, tags, removable] of rows) {
                    if (ssid === cur.ssid) tags.unshift(['接続中', false]);
                    list.appendChild(wifiItem(ssid, tags, removable));
                }
            } catch (e) {
                list.innerHTML = '';
                list.appendChild(Object.assign(document.createElement('li'), { className: 'hint', textContent: 'Wi-Fi 設定の読み込みに失敗しました。' }));
            }
        }

        async function scanWifi() {
            const btn = document.getElementById('wifi_scan_btn');
            const msg = document.getElementById('wifi_message');
            btn.disabled = true; btn.textContent = 'スキャン中…';
            try {
                const res = await fetch('/api/wifi/scan');
                const body = await res.json();
                const dl = document.getElementById('wifi_scan_results');
                dl.innerHTML = '';
                if (!body.ok) { msg.textContent = body.error; return; }
                for (const n of body.networks) {
                    const opt = document.createElement('option');
                    opt.value = n.ssid;
                    opt.label = n.signal_dbm != null ? `${n.signal_dbm} dBm` : '';
                    dl.appendChild(opt);
                }
                msg.textContent = `${body.networks.length} 件見つかりました。SSID 欄で選択できます。`;
            } catch (e) {
                msg.textContent = 'スキャンに失敗しました。';
            } finally {
                btn.disabled = false; btn.textContent = '周辺をスキャン';
            }
        }

        async function addWifi() {
            const ssid = document.getElementById('wifi_ssid').value;
            const password = document.getElementById('wifi_password').value;
            const msg = document.getElementById('wifi_message');
            msg.textContent = '設定を書き込んでいます…';
            const res = await fetch('/api/wifi', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ ssid, password })
            });
            const body = await res.json();
            msg.textContent = body.ok ? body.message : ('追加できませんでした: ' + body.error);
            if (body.ok) {
                document.getElementById('wifi_password').value = '';
                setTimeout(loadWifi, 1000);
            }
        }

        async function removeWifi(ssid, confirmed = false) {
            const msg = document.getElementById('wifi_message');
            const res = await fetch('/api/wifi/remove', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({ ssid, confirm: confirmed })
            });
            const body = await res.json();
            if (body.needs_confirm) {
                if (confirm(body.error + '\\n本当に削除しますか？')) return removeWifi(ssid, true);
                return;
            }
            msg.textContent = body.ok ? body.message : ('削除できませんでした: ' + body.error);
            if (body.ok) setTimeout(loadWifi, 1000);
        }

        // --- カメラ設定 ---
        async function loadCameraConfig() {
            try {
                const res = await fetch('/api/camera');
                const { settings, options } = await res.json();
                const fill = (id, items, toValue, toLabel) => {
                    document.getElementById(id).innerHTML = items
                        .map(it => `<option value="${toValue(it)}">${toLabel(it)}</option>`).join('');
                };
                fill('cam_capture_res', options.capture_resolutions, r => `${r[0]}x${r[1]}`, r => `${r[0]} × ${r[1]}`);
                fill('cam_capture_fps', options.fps, f => f, f => `${f} fps`);
                fill('cam_stream_width', options.stream_widths, w => w, w => `${w} px`);
                fill('cam_stream_fps', options.fps, f => f, f => `${f} fps`);

                document.getElementById('cam_capture_res').value = `${settings.capture_width}x${settings.capture_height}`;
                document.getElementById('cam_capture_fps').value = settings.capture_fps;
                document.getElementById('cam_stream_width').value = settings.stream_width;
                document.getElementById('cam_stream_fps').value = settings.stream_fps;
                updateCameraOptionStates();
            } catch(e) {
                document.getElementById('cam_message').innerText = "カメラ設定の読み込みに失敗しました。";
            }
        }

        // 配信は撮影以下に制限: 超える選択肢を無効化し、選択中なら上限に下げる
        function updateCameraOptionStates() {
            const captureWidth = parseInt(document.getElementById('cam_capture_res').value.split('x')[0]);
            const captureFps = parseInt(document.getElementById('cam_capture_fps').value);
            const limit = (id, max) => {
                const sel = document.getElementById(id);
                let best = null;
                for (const opt of sel.options) {
                    opt.disabled = parseInt(opt.value) > max;
                    if (!opt.disabled) best = opt.value;
                }
                if (sel.selectedOptions[0]?.disabled && best !== null) sel.value = best;
            };
            limit('cam_stream_width', captureWidth);
            limit('cam_stream_fps', captureFps);
        }

        async function saveCameraConfig() {
            const [w, h] = document.getElementById('cam_capture_res').value.split('x').map(Number);
            const data = {
                capture_width: w, capture_height: h,
                capture_fps: Number(document.getElementById('cam_capture_fps').value),
                stream_width: Number(document.getElementById('cam_stream_width').value),
                stream_fps: Number(document.getElementById('cam_stream_fps').value),
            };
            const msg = document.getElementById('cam_message');
            const res = await fetch('/api/camera', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(data)
            });
            const body = await res.json();
            if (res.ok) {
                msg.innerText = body.camera_restarted
                    ? "保存しました。カメラを再起動して撮影設定を反映しています（数秒）。"
                    : "保存しました。";
            } else {
                msg.innerText = "保存に失敗しました: " + (body.error || res.status);
            }
        }

        async function restartSystem() {
            if(!confirm("システム（Pythonプロセス）を再起動します。よろしいですか？")) return;
            const res = await fetch('/api/restart', { method: 'POST' });
            if(res.ok) {
                alert("再起動命令を送信しました。3秒ほど待ってからページをリロードしてください。");
                setTimeout(() => location.reload(), 3000);
            }
        }

        // --- 実行開始 ---
        // 設定は最初の一回だけ読み込む（これで入力がリセットされなくなる）
        loadInitialConfig();
        loadCameraConfig();
        loadWifi();

        // ステータス（センサー値）だけを3秒おきに更新
        setInterval(updateStatus, 3000);
        updateStatus(); // 初回実行
    </script>
</body>
</html>
"""

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
        self.ssid, self.strength = get_cached_wifi_info()

        global CURRENT_ROBOT_ID
        CURRENT_ROBOT_ID = robot_id
        
        
        
        self.pcs = set()
        self.webrtc_task = None
        self._webrtc_lock = asyncio.Lock()   # Offer を1つずつ処理する
        self._webrtc_offer_seq = 0           # 受け取った Offer の通し番号 (古い Offer を飛ばす判定に使う)

    def _stop_motors_on_disconnect(self):
        """
        サーバーとの接続が切れたらモーターを止める (遠隔操作中の暴走防止)。
        ただしユーザープログラム実行中は自律動作とみなし、止めない。
        """
        thread = self.script_manager.execution_thread
        if thread and thread.is_alive():
            print("Connection lost, but user program is running. Motors left under program control.")
            return
        print("Connection lost. Stopping motors for safety.")
        if getattr(self.ros_node, "manual_move_deadline", None) is not None:
            # 手動操作で走行中だった場合のみ通知 (再接続後に Web UI へ届く)
            emit_robot_event("robot_event", "サーバーとの接続が切れたためモーターを停止しました", level="warning")
        self.ros_node.command_queue.put({"command": "move", "left": 0, "right": 0})

    async def forward_events(self, websocket):
        """イベントバス (emit_robot_event) に積まれた通知をサーバーへ送り続ける"""
        while True:
            try:
                event = robot_event_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.05)
                continue
            await websocket.send(json.dumps(event))

    async def send_ack(self, websocket, command_data, ok, message):
        """
        Web UI からの指示に対する結果を返す。request_id が付いた指示のみ (古い Web UI には送らない)
        """
        request_id = command_data.get("request_id")
        if request_id is None:
            return
        await websocket.send(json.dumps({
            "type": "ack", "request_id": request_id, "command": command_data.get("command"),
            "ok": bool(ok), "message": message,
        }))

    async def run(self):
        """WebSocket接続を開始し、送受信タスクを並行実行 切断時は自動リトライ"""
        while True:
            try:
                # ping 5秒間隔 / 応答待ち10秒: 無線が途切れた場合も最大15秒程度で切断を検知する
                async with websockets.connect(self.uri, ping_interval=5, ping_timeout=10) as websocket:
                    # 接続成功後の処理
                    print(f"Connected to server: {self.uri}")
                    # oledに接続成功を表示
                    update_oled(text_lines=["", "Connected!", "", ""], clear=True, start_x=5, start_y=5)
                    await asyncio.sleep(2) # 2秒表示してから通常のステータス表示に戻す
                    update_oled(mode="connection")
                    listen_task = asyncio.create_task(self.listen_for_commands(websocket))
                    send_task = asyncio.create_task(self.send_sensor_data(websocket))
                    event_task = asyncio.create_task(self.forward_events(websocket))

                    # どれかのタスクがエラーで終了するまで待機
                    done, pending = await asyncio.wait(
                        [listen_task, send_task, event_task],
                        return_when=asyncio.FIRST_COMPLETED
                    )
                    
                    # 残ったタスクをキャンセル
                    for task in pending:
                        task.cancel()
                    #await asyncio.gather(listen_task, send_task)
                self._stop_motors_on_disconnect()
            except asyncio.CancelledError:
                # プログラム自体の終了要求ならループを抜ける
                break

            except Exception as e:
                self._stop_motors_on_disconnect()
                print(f"Connection failed: {e}. Retrying in 20 seconds...")
                #oledに接続失敗を表示
                update_oled(text_lines=["", "Connection Failed!", "Retrying...", ""], clear=True, start_x=5, start_y=5)
                await asyncio.sleep(20) # 5秒待ってリトライ
            
            
    async def handle_offer(self, websocket, offer_sdp):
            """ブラウザからのOfferを受け取り、Answerを返す"""
            try:
                print("--- WebRTC: Creating PeerConnection ---")
                self.pc = RTCPeerConnection()
                
                # ビデオトラックを追加
                self.pc.addTrack(ROSCameraTrack(self.ros_node))
                print("--- WebRTC: Added Video Track ---")

                # オファーの設定
                offer = RTCSessionDescription(sdp=offer_sdp, type="offer")
                await self.pc.setRemoteDescription(offer)
                print("--- WebRTC: Remote Description Set ---")

                # アンサーの作成
                answer = await self.pc.createAnswer()
                await self.pc.setLocalDescription(answer)
                print("--- WebRTC: Local Description Created ---")

                # アンサーを送信（ここに送信ログを追加）
                payload = {
                    "type": "webrtc_answer",
                    "sdp": self.pc.localDescription.sdp
                }
                await websocket.send(json.dumps(payload))
                print("--- WebRTC: Answer Sent to Browser ---")

            except Exception as e:
                print(f"!!! WebRTC Error !!! : {e}")
                import traceback
                traceback.print_exc()

    async def listen_for_commands(self, websocket):
        """サーバーからのJSONコマンドを受信して処理"""
        async for message in websocket:
            command_data = {}
            try:
                command_data = json.loads(message)
                command = command_data.get("command")

                 # --- モードチェック ---
                if system_state.get_mode() != "remote":
                    # Remoteモード以外の場合は、上位からのコマンドを無視する (結果だけは返す)
                    await self.send_ack(websocket, command_data, False,
                                        "ロボットがLOCALモードのため指示を受け付けません (ローカルダッシュボードでREMOTEに切り替えてください)")
                    continue

                print(f"Received command: {command_data}")
                if command == "set_robot_id":
                    new_id = command_data.get("new_id")
                    if new_id:
                        print(f"!!! ID Change Requested: {CURRENT_ROBOT_ID} -> {new_id} !!!")
                        
                        # 1. 設定ファイルに保存
                        if ConfigManager.update_robot_id(new_id):
                            # 2. ユーザーに通知 (OLED & ログ)
                            update_oled(text_lines=["", "ID CHANGED!", f"-> {new_id}", "Rebooting..."], clear=True, start_x=0, start_y=0)
                            
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
                    # I2Cスキャンは同期処理なので別スレッドで実行 (イベントループを止めない)
                    devices = await asyncio.to_thread(scan_i2c_bus, bus_num=1)
                    
                    # 結果をフロントに返信
                    response = {
                        "type": "i2c_device_list",
                        "devices": devices
                    }
                    await websocket.send(json.dumps(response))
                    print(f"Sent I2C device list: {devices}")    
                elif command == "save_code":
                    code = command_data.get("code")
                    if code:
                        ok, msg = self.script_manager.save_code(code)
                    else:
                        ok, msg = False, "書き込むプログラムが空です"
                    await self.send_ack(websocket, command_data, ok, msg)
                elif command == "start_program":
                    ok, msg = self.script_manager.start_program()
                    await self.send_ack(websocket, command_data, ok, msg)
                elif command == "stop_program":
                    ok, msg = self.script_manager.stop_program()
                    await self.send_ack(websocket, command_data, ok, msg)
                elif command == "list_log_files":
                    await self.handle_list_log_files(websocket)
                elif command == "get_log_file":
                    filename = command_data.get("filename")
                    if filename: await self.handle_get_log_file(websocket, filename)
                    else:
                        await websocket.send(json.dumps({"type": "error", "message": "'filename' is required."}))
                elif command == "snapshot":
                    await self.handle_snapshot(websocket)
                elif command == "webrtc_offer":
                    # 基地局からのWebRTC接続要求を処理する
                    target_id = command_data.get("target_robot_id") or command_data.get("robot_id")
                    if target_id and target_id != CURRENT_ROBOT_ID:
                        continue
                    sdp = command_data.get("sdp")

                    # 交渉中のタスクはキャンセルしない (ICE 候補収集の途中で接続を閉じると、
                    # aioice が閉じた UDP ソケットに送信し続けて asyncio 内部でエラーになるため)。
                    # 代わりに Offer に通し番号を付け、handle_webrtc_offer 側で順番に処理し古いものは飛ばす
                    self._webrtc_offer_seq += 1
                    self.webrtc_task = asyncio.create_task(self.handle_webrtc_offer(
                        websocket, sdp, self._webrtc_offer_seq, command_data.get("session_id")))
                else:
                    # その他のコマンドはROSノードのコマンドキューへ
                    if command == "move":
                        command_data["source"] = "remote"  # デッドマン監視の対象にする
                    self.ros_node.command_queue.put(command_data)
                    # 実際の処理結果は ROS スレッドから robot_event で通知される
                    await self.send_ack(websocket, command_data, True, "指示を受け付けました")

            except json.JSONDecodeError:
                print(f"Received non-JSON message: {message}")
            except Exception as e:
                print(f"Error processing command: {e}")
                try:
                    await self.send_ack(websocket, command_data, False, f"ロボット側でエラーが発生しました: {e}")
                except Exception:
                    pass
                
    async def handle_webrtc_offer(self, websocket, sdp, offer_seq=None, session_id=None):
        """
        WebRTCのOfferを受け取り、Answerを返すシグナリング処理。
        Offer は1つずつ順番に処理し (ロック)、待っている間により新しい Offer が来ていれば交渉せずに飛ばす。
        session_id は Web UI が付けた番号で、Answer にそのまま付けて返す (Web UI が古い Answer を無視するため)。
        """
        async with self._webrtc_lock:
            if offer_seq is not None and offer_seq != self._webrtc_offer_seq:
                print(f"Skipped stale WebRTC offer (#{offer_seq}, latest #{self._webrtc_offer_seq}).")
                return
            await self._negotiate_webrtc(websocket, sdp, session_id)

    async def _negotiate_webrtc(self, websocket, sdp, session_id):
        print("Received WebRTC Offer. Establishing Peer Connection...")
        for old_pc in list(self.pcs):
            try:
                await old_pc.close()
                print("Closed previous PeerConnection.")
            except Exception as e:
                print(f"Error closing old PeerConnection: {e}")
        self.pcs.clear()
        
        ice_servers = [
            RTCIceServer(
                urls=["stun:219.94.244.174:3478"]
            ),
            RTCIceServer(
                urls=[
                    #"turn:219.94.244.174:3478?transport=udp",
                    "turn:219.94.244.174:3478?transport=tcp",
                    #"turn:219.94.244.174:3478"     
                ],
                username="catuser",
                credential="catpassword"
            )
        ]
        
        config = RTCConfiguration(iceServers=ice_servers)
        # 新しいピア接続を作成
        pc = RTCPeerConnection(configuration=config)
        self.pcs.add(pc)
        print("Created new RTCPeerConnection for WebRTC session.{Current PC count: " + str(len(self.pcs)) + "}")

        # 接続状態の監視
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"WebRTC Connection State is {pc.connectionState}")
            #print(pc.localDescription.sdp)
            if pc.connectionState in["failed", "closed"]:
                self.pcs.discard(pc)
        @pc.on("icecandidate")
        def on_icecandidate(candidate):
            print("New ICE candidate gathered:")
            print(candidate)
        # @pc.on("icecandidate")
        # async def on_icecandidate(candidate):
        #     if candidate:
        #         await websocket.send(json.dumps({
        #             "type": "candidate",
        #             "candidate": candidate.to_sdp()
        #         }))
        #         print("Sent ICE candidate to server:")
        #         print(candidate)
        @pc.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            print("ICE Connection State:", pc.iceConnectionState)

        # 用意されているカメラトラックをPeerConnectionに追加
        pc.addTrack(ROSCameraTrack(self.ros_node))

        try:
            # 基地局からのOfferをリモート情報としてセット
            offer = RTCSessionDescription(sdp=sdp, type="offer")
            await pc.setRemoteDescription(offer)

            # ロボット側のAnswerを作成してローカル情報としてセット
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            
            # 修正ポイント 3: タイムアウト3秒でICE gatheringが完了するのを待つ
            timeout = 3.0
            start_time = asyncio.get_event_loop().time()
            while pc.iceGatheringState != "complete":
                await asyncio.sleep(0.1)
                if asyncio.get_event_loop().time() - start_time > timeout:
                    print("!!! WebRTC: ICE gathering timed out, sending partial SDP !!!")
                    break
                    
            # WebSocket経由で基地局にAnswerを返信
            response = {
                "type": "webrtc_answer",
                "sdp": pc.localDescription.sdp,
                "session_id": session_id,
            }
            await websocket.send(json.dumps(response))
            print("Sent WebRTC Answer.")

        except Exception as e:
            print(f"WebRTC Negotiation Error: {e}")
            traceback.print_exc()
            self.pcs.discard(pc)
            try:
                await pc.close()  # 失敗した接続を残さない
            except Exception:
                pass
            emit_robot_event("robot_event", f"映像の接続に失敗しました: {e}", level="error")
            
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
            payload["data"]["gps"] = gps_to_dict(gps_msg)
            
            if bme_read: payload["data"]["bme280"] = bme280_to_dict(bme_read)
            else: payload["data"]["bme280"] = None
                
            ssid, strength = get_cached_wifi_info()  # 30Hzループなのでキャッシュを参照
            cpu_temp = get_cpu_temperature() 
            with system_info_lock:
                if ssid and strength:
                    latest_system_info['wifi_ssid'] = ssid
                    latest_system_info['wifi_strength'] = strength
                latest_system_info['cpu_temp'] = cpu_temp 

            payload["data"]["wifi"] = {"ssid": ssid, "signal_strength": strength}
            payload["data"]["cpu_temperature"] = cpu_temp

            if payload["data"]:
                await websocket.send(json.dumps(payload))

            await asyncio.sleep(0.033)  # 約30fps

    def _encode_snapshot(self):
        """
        最新のカメラ画像を撮影解像度のまま正立させてJPEG化する。
        戻り値: (jpeg_bytes, width, height)。画像がない/古い場合は ValueError
        """
        with image_lock:
            msg = latest_image_msg
            age = time.monotonic() - latest_image_time
        if msg is None or age > SNAPSHOT_MAX_IMAGE_AGE_SEC:
            raise ValueError("カメラ画像がありません。カメラをONにしてください。")
        cv_img = cv2.flip(self.ros_node.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), -1)
        ok, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, SNAPSHOT_JPEG_QUALITY])
        if not ok:
            raise ValueError("JPEGへの変換に失敗しました。")
        h, w = cv_img.shape[:2]
        return buf.tobytes(), w, h

    async def handle_snapshot(self, websocket):
        """スナップショットを撮影解像度のJPEG(base64)で返す"""
        try:
            # 変換・エンコードは重いので別スレッドで (イベントループを止めない)
            jpeg, w, h = await asyncio.to_thread(self._encode_snapshot)
        except Exception as e:
            await websocket.send(json.dumps({"type": "error", "context": "snapshot", "message": str(e)}))
            return
        filename = f"{CURRENT_ROBOT_ID}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        await websocket.send(json.dumps({
            "type": "snapshot", "filename": filename, "width": w, "height": h,
            "data": base64.b64encode(jpeg).decode('ascii'),
        }))
        print(f"Sent snapshot {filename} ({w}x{h}, {len(jpeg)//1024} KB)")

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
    if OLED_INIT_ERROR:
        # サーバー接続後に Web UI のコンソールへ届く
        emit_robot_event("robot_event", f"{OLED_INIT_ERROR}。OLED 表示なしで動作しています。", level="warning")

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

    # Wi-Fi情報の定期取得スレッド
    threading.Thread(target=_wifi_monitor_loop, daemon=True).start()
    
    # ROSスレッド
    ros_thread = threading.Thread(target=run_ros_spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # WebSocketクライアント起動
    client = RobotWebsocketClient(
        ros_node=ros_node, 
        script_manager=script_manager,
        robot_id=CURRENT_ROBOT_ID, # Configから読み込んだID
        server_uri=TARGET_URI
    )
     #  全ての非同期タスクを管理するエントリポイント
    async def main_loop():
        # WebSocketタスク
        ws_task = asyncio.create_task(client.run())
        
        # FastAPIサーバー (uvicorn) タスク
        # host="0.0.0.0" にすることでLAN内の他PCからアクセス可能に
        config_uvicorn = uvicorn.Config(app, host="0.0.0.0", port=8080, log_level="info")
        server = uvicorn.Server(config_uvicorn)
        web_task = asyncio.create_task(server.serve())
        
        await asyncio.gather(ws_task, web_task)
        
        
    try:
        print("Starting WebSocket client...")
        asyncio.run(main_loop())

    except KeyboardInterrupt:
        print("Application stopped by user (Ctrl+C).")
        
    finally:
        print("Shutting down rclpy...")
        
        if 'client' in locals() and hasattr(client, 'pcs'):
            for pc in list(client.pcs): # list()でコピーして安全に回す
                try:
                    pc.close()
                except Exception as e:
                    print(f"Error closing WebRTC: {e}")
            client.pcs.clear()
        rclpy.shutdown()
        ros_thread.join(timeout=2)
        try:
            if 'client' in locals() and hasattr(client, 'bme280') and client.bme280:
                client.bme280.close()
        except Exception: pass
        update_oled(clear=True)
        
        print("Application has exited.")