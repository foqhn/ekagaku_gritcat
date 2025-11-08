import gpsd
import time

try:
    # gpsdに接続
    gpsd.connect()
    print("gpsdに接続しました。データ受信待機中...")

    while True:
        # 最新のGPS情報を取得
        packet = gpsd.get_current()

        # 測位できているか確認
        if packet.mode >= 2:
            print("========================================")
            print(f"時刻: {packet.time}")
            print(f"緯度: {packet.lat}")
            print(f"経度: {packet.lon}")
            print(f"高度: {packet.alt} m")
            print(f"進行方向: {packet.track} 度")
            print(f"水平速度: {packet.hspeed} m/s")
            print(f"垂直速度: {packet.climb} m/s")
            print(f"捕捉衛星数: {packet.sats_valid} / {packet.sats}")
            
            # 'error' 辞書が存在するか確認してから、中のデータにアクセス
            if hasattr(packet, 'error') and isinstance(packet.error, dict):
                print("---------- 精度情報 ----------")
                # .get(key, default_value) を使うとキーが無くてもエラーにならない
                lat_err = packet.error.get('y', 'N/A')
                lon_err = packet.error.get('x', 'N/A')
                alt_err = packet.error.get('v', 'N/A')
                
                print(f"緯度誤差: ±{lat_err} m")
                print(f"経度誤差: ±{lon_err} m")
                print(f"高度誤差: ±{alt_err} m")
            
        else:
            print("測位待機中...")

        time.sleep(1)

except KeyboardInterrupt:
    print("\nプログラムを終了します。")
except Exception as e:
    print(f"エラーが発生しました: {e}")