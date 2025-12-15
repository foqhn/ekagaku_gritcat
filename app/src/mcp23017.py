import lgpio
import time

class MCP23017:
    # レジスタアドレス定義 (IOCON.BANK = 0 を想定)
    REG_IODIRA = 0x00  # 方向レジスタ A (0=Output, 1=Input)
    REG_IODIRB = 0x01  # 方向レジスタ B
    REG_GPPUA  = 0x0C  # プルアップ抵抗 A
    REG_GPPUB  = 0x0D  # プルアップ抵抗 B
    REG_GPIOA  = 0x12  # 入力ポート A
    REG_GPIOB  = 0x13  # 入力ポート B
    REG_OLATA  = 0x14  # 出力ラッチ A
    REG_OLATB  = 0x15  # 出力ラッチ B

    # 定数
    INPUT = 1
    OUTPUT = 0
    HIGH = 1
    LOW = 0

    def __init__(self, bus=1, address=0x20):
        """
        初期化
        :param bus: I2Cバス番号 (Raspberry Pi 4は通常 1)
        :param address: MCP23017のI2Cアドレス (デフォルト 0x20)
        """
        self.i2c_handle = lgpio.i2c_open(bus, address)
        self.address = address

    def _read_byte(self, reg):
        """指定レジスタから1バイト読み込む"""
        return lgpio.i2c_read_byte_data(self.i2c_handle, reg)

    def _write_byte(self, reg, value):
        """指定レジスタへ1バイト書き込む"""
        lgpio.i2c_write_byte_data(self.i2c_handle, reg, value)

    def _update_register_bit(self, reg, bit, value):
        """
        Read-Modify-Writeを行い、特定のビットだけを変更する
        """
        current_val = self._read_byte(reg)
        if value:
            new_val = current_val | (1 << bit)
        else:
            new_val = current_val & ~(1 << bit)
        self._write_byte(reg, new_val)

    def _get_port_params(self, pin):
        """
        ピン番号(0-15)から、ポート(A/B)に対応するレジスタオフセットとビット位置を計算
        Pin 0-7 -> Port A, Pin 8-15 -> Port B
        """
        if 0 <= pin <= 7:
            return 0, pin  # offset (Port A), bit
        elif 8 <= pin <= 15:
            return 1, pin - 8  # offset (Port B), bit
        else:
            raise ValueError("Pin number must be 0-15")

    def setup(self, pin, mode, pull_up=False):
        """
        ピンのモード設定 (INPUT / OUTPUT)
        :param pin: 0-15
        :param mode: MCP23017.INPUT or MCP23017.OUTPUT
        :param pull_up: Trueの場合、内部プルアップ抵抗を有効にする(Input時のみ)
        """
        offset, bit = self._get_port_params(pin)
        
        # 方向レジスタの設定 (IODIRA/IODIRB)
        reg_iodir = self.REG_IODIRA + offset
        self._update_register_bit(reg_iodir, bit, mode)

        # プルアップの設定 (GPPUA/GPPUB)
        if mode == self.INPUT and pull_up:
            reg_gppu = self.REG_GPPUA + offset
            self._update_register_bit(reg_gppu, bit, 1)
        else:
            # Outputモードやプルアップ不要な場合はOFFにする
            reg_gppu = self.REG_GPPUA + offset
            self._update_register_bit(reg_gppu, bit, 0)

    def output(self, pin, value):
        """
        出力ピンの状態を設定 (HIGH / LOW)
        :param pin: 0-15
        :param value: 1 (HIGH) or 0 (LOW)
        """
        offset, bit = self._get_port_params(pin)
        # 出力ラッチレジスタ (OLATA/OLATB) を使用
        reg_olat = self.REG_OLATA + offset
        self._update_register_bit(reg_olat, bit, 1 if value else 0)

    def input(self, pin):
        """
        入力ピンの状態を読み取る
        :param pin: 0-15
        :return: 1 (HIGH) or 0 (LOW)
        """
        offset, bit = self._get_port_params(pin)
        # 入力ポートレジスタ (GPIOA/GPIOB) を使用
        reg_gpio = self.REG_GPIOA + offset
        val = self._read_byte(reg_gpio)
        return (val >> bit) & 1

    def cleanup(self):
        """I2C接続を閉じる"""
        try:
            lgpio.i2c_close(self.i2c_handle)
        except:
            pass


def main():
    # インスタンス作成 (アドレスは環境に合わせて変更してください: デフォルト0x20)
    mcp = MCP23017(bus=1, address=0x20)

    try:
        print("MCP23017 Test Start")

        # Pin 0 を出力に設定 (LED用)
        mcp.setup(0, MCP23017.OUTPUT)

        # Pin 8 を入力（プルアップ有効）に設定 (スイッチ用)
        # ボタンを押すとGNDに落ちる回路を想定
        mcp.setup(8, MCP23017.INPUT, pull_up=True)

        while True:
            # スイッチの状態を読み取る
            # プルアップしているので、押されていない=1, 押された=0
            btn_state = mcp.input(8)

            if btn_state == 0:
                print("Button Pressed!")
                # ボタンが押されたら高速点滅
                delay = 0.1
            else:
                # 押されていないならゆっくり点滅
                delay = 0.5

            # LED点灯
            mcp.output(0, MCP23017.HIGH)
            time.sleep(delay)

            # LED消灯
            mcp.output(0, MCP23017.LOW)
            time.sleep(delay)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # 終了処理
        mcp.output(0, MCP23017.LOW) # LEDを消しておく
        mcp.cleanup()

if __name__ == "__main__":
    main()