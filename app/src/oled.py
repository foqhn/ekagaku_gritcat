# -*- coding: utf-8 -*-

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont

class OLEDDisplay:
    """
    luma.oledライブラリを使用してSSD1306 OLEDディスプレイを制御するクラス。
    """

    def __init__(self, port=1, address=0x3C, width=128, height=64, font_path="arial.ttf", font_size=15):
        """
        OLEDディスプレイを初期化します。

        Args:
            port (int): I2Cバスの番号 (Raspberry Piでは通常1)。
            address (hex): OLEDディスプレイのI2Cアドレス (通常は0x3C)。
            width (int): ディスプレイの幅（ピクセル）。
            height (int): ディスプレイの高さ（ピクセル）。
            font_path (str): 使用するTrueTypeフォントのパス。
            font_size (int): フォントサイズ。
        """
        try:
            # I2Cインターフェースの設定
            self.serial = i2c(port=port, address=address)
            # 使用するOLEDディスプレイのドライバと解像度を指定
            self.device = ssd1306(self.serial, width=width, height=height)
            # フォントの読み込み
            self.font = ImageFont.truetype(font_path, font_size)
        except IOError:
            print(f"警告: フォント '{font_path}' が見つかりません。デフォルトフォントを使用します。")
            self.font = ImageFont.load_default()
        except Exception as e:
            print(f"エラー: OLEDディスプレイの初期化に失敗しました: {e}")
            print("I2Cの設定や接続を確認してください。 `sudo i2cdetect -y 1` コマンドが役立ちます。")
            raise
        self._last_lines = None

    def display_text(self, text_lines, start_x=5, start_y=5, line_spacing=20):
        """
        OLEDに複数行のテキストを表示します。
        前回と同じ内容の場合は描画をスキップします。

        Args:
            text_lines (list): 表示したい文字列を要素とするリスト。
            start_x (int): 描画を開始するX座標。
            start_y (int): 最初の行の描画を開始するY座標。
            line_spacing (int): 各行の縦方向の間隔。
        """
        # 内容が前回と全く同じなら、再描画せず終了（ちらつき防止の最強策）
        if text_lines == self._last_lines:
            return

        # 新しい内容を保存
        self._last_lines = text_lines[:]

        # canvasコンテキストに入ると、裏で新しい画像バッファが作られます(へーすげえ)
        with canvas(self.device) as draw:
            # 【修正】draw.rectangleでの黒塗りは不要（canvasは毎回初期化されるため）。
            # ここを削除することで無駄な処理が減ります。

            # 各行のテキストを描画
            y = start_y
            for line in text_lines:
                draw.text((start_x, y), str(line), font=self.font, fill="white")
                y += line_spacing

    def clear(self):
        """
        ディスプレイの表示をすべて消去します。
        """
        self.device.clear()
        self._last_lines = None  # キャッシュもクリア