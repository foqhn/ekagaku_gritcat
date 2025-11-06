import subprocess
import re

def get_wifi_info(interface='wlan0'):
    """
    指定されたワイヤレスインターフェースのSSIDと信号強度を取得します。

    Args:
        interface (str): ネットワークインターフェース名（デフォルトは 'wlan0'）。

    Returns:
        tuple: (ssid, signal_level) のタプル。
               接続されていない、または情報を取得できない場合は (None, None)。
    """
    try:
        # iwconfigコマンドを実行し、出力をUTF-8でデコード
        # stderr=subprocess.STDOUTを追加して、エラー出力もキャプチャする
        scan_output = subprocess.check_output(
            ['iwconfig', interface],
            stderr=subprocess.STDOUT
        ).decode('utf-8')

        # 正規表現を使ってSSIDを抽出
        ssid_match = re.search(r'ESSID:"(.+?)"', scan_output)
        ssid = ssid_match.group(1) if ssid_match else None

        # 正規表現を使って信号レベルを抽出
        signal_match = re.search(r'Signal level=(.+?) dBm', scan_output)
        signal_level = signal_match.group(1) if signal_match else None

        # Link Qualityから取得する fallback
        if not signal_level:
            quality_match = re.search(r'Link Quality=(\d+/\d+)', scan_output)
            if quality_match:
                signal_level = quality_match.group(1)

        return (ssid, signal_level)

    except subprocess.CalledProcessError as e:
        # コマンド実行に失敗した場合（例：wlan0が存在しない）
        # print(f"Error executing iwconfig: {e}")
        return (None, None)
    except Exception as e:
        # その他のエラー
        # print(f"An error occurred: {e}")
        return (None, None)

if __name__ == '__main__':
    ssid, strength = get_wifi_info()

    if ssid and strength:
        print(f"接続中のSSID: {ssid}")
        print(f"電波強度: {strength} dBm")
    else:
        print("Wi-Fiに接続されていないか、情報を取得できませんでした。")