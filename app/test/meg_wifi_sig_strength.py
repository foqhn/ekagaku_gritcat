import subprocess
import re
import os
import time
import statistics
import speedtest
import csv
from datetime import datetime

def get_wifi_info(interface='wlan0'):
    """指定されたワイヤレスインターフェースのSSIDと信号強度を取得します。"""
    try:
        scan_output = subprocess.check_output(
            ['iwconfig', interface],
            stderr=subprocess.STDOUT,
            timeout=1.5
        ).decode('utf-8')

        ssid_match = re.search(r'ESSID:"(.+?)"', scan_output)
        ssid = ssid_match.group(1) if ssid_match else "N/A"

        signal_match = re.search(r'Signal level=(.+?) dBm', scan_output)
        signal_level = signal_match.group(1) if signal_match else "N/A"

        if signal_level == "N/A":
            quality_match = re.search(r'Link Quality=(\d+/\d+)', scan_output)
            if quality_match:
                signal_level = quality_match.group(1)

        return (ssid, signal_level)
    except Exception:
        return ("N/A", "N/A")

def get_cpu_temperature():
    """CPU温度を取得します。"""
    try:
        temp_path = "/sys/class/hwmon/hwmon0/temp1_input"
        if os.path.exists(temp_path):
            with open(temp_path, "r") as f:
                temp_milli = float(f.read().strip())
                return temp_milli / 1000.0
        return 0.0
    except Exception:
        return 0.0

def measure_signal_stats(duration=15, interval=1.0, interface='wlan0'):
    """
    指定した時間だけ電波強度を測定し、平均と分散を計算します。
    戻り値: (平均値, 分散値) ※取得失敗時は (None, None)
    """
    start_time = time.time()
    signal_levels = []
    current_ssid = "N/A"
    
    print(f"\n[{interface}] {duration}秒間の電波強度測定を開始します... (約{interval}秒間隔)")
    
    while time.time() - start_time < duration:
        ssid, signal = get_wifi_info(interface)
        
        if ssid != "N/A":
            current_ssid = ssid
            
        try:
            if signal != "N/A":
                val = float(signal)
                signal_levels.append(val)
                print(f"電波取得: {val} dBm")
        except ValueError:
            pass
            
        time.sleep(interval)

    print("-" * 40)
    print(f"SSID: {current_ssid} | 測定回数: {len(signal_levels)} 回")
    
    if len(signal_levels) > 0:
        mean_val = statistics.mean(signal_levels)
        var_val = statistics.variance(signal_levels) if len(signal_levels) > 1 else 0.0
        print(f"電波強度 平均: {mean_val:.2f} dBm")
        print(f"電波強度 分散: {var_val:.2f}")
        return mean_val, var_val
    else:
        print("有効な電波強度のデータが取得できませんでした。")
        return None, None

def measure_internet_speed():
    """
    Speedtestを利用してインターネット速度を測定します。
    戻り値: (ダウンロード速度[Mbps], アップロード速度[Mbps], Ping[ms])
    """
    print("\nインターネット速度を測定中... (この処理には数十秒かかります)")
    try:
        st = speedtest.Speedtest()
        
        print("最適なサーバーを検索中...")
        st.get_best_server()
        
        print("ダウンロード速度を測定中...")
        download_mbps = st.download() / 1_000_000
        
        print("アップロード速度を測定中...")
        upload_mbps = st.upload() / 1_000_000
        
        ping = st.results.ping
        
        return download_mbps, upload_mbps, ping

    except Exception as e:
        print(f"測定エラー: {e}")
        return None, None, None

def save_to_csv(filename, mean, var, ping, dl, ul):
    """測定結果をCSVファイルに書き込みます"""
    # ファイルが存在するか確認（ヘッダー書き込みの判定用）
    file_exists = os.path.isfile(filename)
    
    # utf-8-sig にすることで、WindowsのExcelで開いた際の文字化けを防ぎます
    with open(filename, mode='a', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        
        # ファイルが新規作成される場合はヘッダーを書き込む
        if not file_exists:
            writer.writerow(['測定日時', '電波平均(dBm)', '電波分散', 'Ping(ms)', 'Download(Mbps)', 'Upload(Mbps)'])
        
        # エラーでNoneが入っている場合は "N/A" を書き込む処理
        row = [
            datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            f"{mean:.2f}" if mean is not None else "N/A",
            f"{var:.2f}"  if var is not None else "N/A",
            f"{ping:.2f}" if ping is not None else "N/A",
            f"{dl:.2f}"   if dl is not None else "N/A",
            f"{ul:.2f}"   if ul is not None else "N/A"
        ]
        writer.writerow(row)

if __name__ == "__main__":
    csv_filename = "wifi_data.csv"
    print("=== Wi-Fi & インターネット速度 連続測定ツール ===")
    
    while True:
        # 1. 電波強度の測定
        mean_val, var_val = measure_signal_stats(duration=15, interval=1.0)
        
        # 2. インターネット速度測定
        dl_speed, ul_speed, ping_val = measure_internet_speed()
        
        if dl_speed is not None:
            print("-" * 40)
            print("インターネット速度 測定結果")
            print(f"Ping     : {ping_val:.2f} ms")
            print(f"Download : {dl_speed:.2f} Mbps")
            print(f"Upload   : {ul_speed:.2f} Mbps")
            print("-" * 40)

        # 3. CSVへの保存
        save_to_csv(csv_filename, mean_val, var_val, ping_val, dl_speed, ul_speed)
        print(f"\n✅ 測定結果を [{csv_filename}] に保存しました。")

        # 4. CPU温度の表示 (おまけ情報としてコンソールにだけ出す)
        cpu_temp = get_cpu_temperature()
        print(f"現在のCPU温度: {cpu_temp:.2f} °C")

        # 5. ユーザー入力による待機・終了分岐
        print("\n" + "=" * 50)
        user_input = input("▶ Enterキーを押すと次の測定を開始します。(終了する場合は 'q' と入力): ")
        
        # 'q' または 'Q' が入力されたらループを抜けて終了
        if user_input.strip().lower() == 'q':
            print("測定を終了します。お疲れ様でした！")
            break