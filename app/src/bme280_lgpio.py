#coding: utf-8
import lgpio
import time

class BME280:
    def __init__(self, bus_number=1, i2c_address=0x76):
        self.bus_number = bus_number
        self.i2c_address = i2c_address
        self.h = None
        
        self.digT = []
        self.digP = []
        self.digH = []
        self.t_fine = 0.0

        # 初期化処理
        self.open()
        self._setup_sensor()
        self._get_calib_param()

    def open(self):
        """I2C接続を開く"""
        if self.h is None:
            try:
                self.h = lgpio.i2c_open(self.bus_number, self.i2c_address)
            except lgpio.error as e:
                raise RuntimeError(f"Failed to open I2C bus: {e}")

    def close(self):
        """I2C接続を閉じる"""
        if self.h is not None:
            lgpio.i2c_close(self.h)
            self.h = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _write_reg(self, reg_address, data):
        lgpio.i2c_write_byte_data(self.h, reg_address, data)

    def _read_byte(self, reg_address):
        return lgpio.i2c_read_byte_data(self.h, reg_address)

    def _setup_sensor(self):
        """センサの設定を書き込む"""
        osrs_t = 1          # Temperature oversampling x 1
        osrs_p = 1          # Pressure oversampling x 1
        osrs_h = 1          # Humidity oversampling x 1
        mode   = 3          # Normal mode
        t_sb   = 5          # Tstandby 1000ms
        filter = 0          # Filter off
        spi3w_en = 0        # 3-wire SPI Disable

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        self._write_reg(0xF2, ctrl_hum_reg)
        self._write_reg(0xF4, ctrl_meas_reg)
        self._write_reg(0xF5, config_reg)

    def _get_calib_param(self):
        """補正データを読み込む"""
        calib = []
        
        for i in range(0x88, 0x88+24):
            calib.append(self._read_byte(i))
        calib.append(self._read_byte(0xA1))
        for i in range(0xE1, 0xE1+7):
            calib.append(self._read_byte(i))

        self.digT.append((calib[1] << 8) | calib[0])
        self.digT.append((calib[3] << 8) | calib[2])
        self.digT.append((calib[5] << 8) | calib[4])
        self.digP.append((calib[7] << 8) | calib[6])
        self.digP.append((calib[9] << 8) | calib[8])
        self.digP.append((calib[11]<< 8) | calib[10])
        self.digP.append((calib[13]<< 8) | calib[12])
        self.digP.append((calib[15]<< 8) | calib[14])
        self.digP.append((calib[17]<< 8) | calib[16])
        self.digP.append((calib[19]<< 8) | calib[18])
        self.digP.append((calib[21]<< 8) | calib[20])
        self.digP.append((calib[23]<< 8) | calib[22])
        self.digH.append(calib[24])
        self.digH.append((calib[26]<< 8) | calib[25])
        self.digH.append(calib[27])
        self.digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        self.digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append(calib[31])
        
        for i in range(1, 2):
            if self.digT[i] & 0x8000:
                self.digT[i] = (-self.digT[i] ^ 0xFFFF) + 1

        for i in range(1, 8):
            if self.digP[i] & 0x8000:
                self.digP[i] = (-self.digP[i] ^ 0xFFFF) + 1

        for i in range(0, 6):
            if self.digH[i] & 0x8000:
                self.digH[i] = (-self.digH[i] ^ 0xFFFF) + 1  

    def read_data(self):
        """
        センサからデータを読み取り、補正後の値を辞書形式で返す
        Return: {'temperature': float, 'pressure': float, 'humidity': float}
        """
        # ブロック読み込みを使って高速化
        # 0xF7から8バイト読み込む
        try:
            (count, data) = lgpio.i2c_read_i2c_block_data(self.h, 0xF7, 8)
        except lgpio.error:
            # エラー時は再試行せずNoneを返す等の処理が可能ですが、ここでは例外を送出
            raise

        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]
        
        # t_fineを更新するために必ず温度から計算する
        temp = self._compensate_T(temp_raw)
        pres = self._compensate_P(pres_raw)
        hum  = self._compensate_H(hum_raw)

        return {
            'temperature': temp,
            'pressure': pres,
            'humidity': hum
        }

    def _compensate_P(self, adc_P):
        pressure = 0.0
        
        v1 = (self.t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768
        
        if v1 == 0:
            return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)  

        return pressure / 100.0  # hPaに変換して返す

    def _compensate_T(self, adc_T):
        v1 = (adc_T / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (adc_T / 131072.0 - self.digT[0] / 8192.0) * (adc_T / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        self.t_fine = v1 + v2
        temperature = self.t_fine / 5120.0
        return temperature

    def _compensate_H(self, adc_H):
        var_h = self.t_fine - 76800.0
        if var_h != 0:
            var_h = (adc_H - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * var_h)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * var_h * (1.0 + self.digH[2] / 67108864.0 * var_h)))
        else:
            return 0
        var_h = var_h * (1.0 - self.digH[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        return var_h

# このファイル自体を実行した時のテストコード
if __name__ == '__main__':
    try:
        # with構文を使うと自動でclose()が呼ばれます
        with BME280(bus_number=1, i2c_address=0x76) as sensor:
            while True:
                data = sensor.read_data()
                print(f"Temp: {data['temperature']:.2f} ℃, "
                      f"Pressure: {data['pressure']:.2f} hPa, "
                      f"Humidity: {data['humidity']:.2f} %")
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")