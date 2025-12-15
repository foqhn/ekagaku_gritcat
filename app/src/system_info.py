import subprocess
import re
import os

def get_wifi_info(interface='wlan0'):
    """
    指定されたワイヤレスインターフェースのSSIDと信号強度を取得します。
    Returns: (ssid, signal_level)
    """
    try:
        scan_output = subprocess.check_output(
            ['iwconfig', interface],
            stderr=subprocess.STDOUT
        ).decode('utf-8')

        ssid_match = re.search(r'ESSID:"(.+?)"', scan_output)
        ssid = ssid_match.group(1) if ssid_match else "N/A"

        signal_match = re.search(r'Signal level=(.+?) dBm', scan_output)
        signal_level = signal_match.group(1) if signal_match else "N/A"

        if not signal_level:
            quality_match = re.search(r'Link Quality=(\d+/\d+)', scan_output)
            if quality_match:
                signal_level = quality_match.group(1)

        return (ssid, signal_level)

    except Exception:
        return ("N/A", "N/A")

def get_cpu_temperature():
    """
    CPU温度を取得します。
    /sys/class/hwmon/hwmon0/temp1_input (ミリ度) を読み取って度(℃)に変換します。
    Returns: float (温度) or 0.0 (エラー時)
    """
    try:
        # 多くのLinux環境(Raspberry Pi含む)での標準的なパス
        # temp?_input の ? は通常 1 ですが、環境によって異なる場合は調整してください
        temp_path = "/sys/class/hwmon/hwmon0/temp1_input"
        
        if os.path.exists(temp_path):
            with open(temp_path, "r") as f:
                # 値はミリ度セルシウスなので 1000 で割る
                temp_milli = float(f.read().strip())
                return temp_milli / 1000.0
        return 0.0
    except Exception:
        return 0.0