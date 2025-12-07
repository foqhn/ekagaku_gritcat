#!/usr/bin/python

# MIT License
# ... (Copyright notice remains same) ...

from ctypes import CDLL, CFUNCTYPE, POINTER, c_int, c_uint, pointer, c_ubyte, c_uint8, c_uint32
import sysconfig
import pkg_resources
import site
import lgpio  # lgpioに変更
import time
import os

class Vl53l0xError(RuntimeError):
    pass


class Vl53l0xAccuracyMode:
    GOOD = 0        # 33 ms timing budget 1.2m range
    BETTER = 1      # 66 ms timing budget 1.2m range
    BEST = 2        # 200 ms 1.2m range
    LONG_RANGE = 3  # 33 ms timing budget 2m range
    HIGH_SPEED = 4  # 20 ms timing budget 1.2m range


class Vl53l0xDeviceMode:
    SINGLE_RANGING = 0
    CONTINUOUS_RANGING = 1
    SINGLE_HISTOGRAM = 2
    CONTINUOUS_TIMED_RANGING = 3
    SINGLE_ALS = 10
    GPIO_DRIVE = 20
    GPIO_OSC = 21


class Vl53l0xGpioAlarmType:
    OFF = 0
    THRESHOLD_CROSSED_LOW = 1
    THRESHOLD_CROSSED_HIGH = 2
    THRESHOLD_CROSSED_OUT = 3
    NEW_MEASUREMENT_READY = 4


class Vl53l0xInterruptPolarity:
    LOW = 0
    HIGH = 1


# Read/write function pointer types.
_I2C_READ_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)
_I2C_WRITE_FUNC = CFUNCTYPE(c_int, c_ubyte, c_ubyte, POINTER(c_ubyte), c_ubyte)

# Load VL53L0X shared lib

suffix = sysconfig.get_config_var('EXT_SUFFIX')
if suffix is None:
    suffix = ".so"

# 探すファイル名の候補（OS固有のサフィックス付き と 単純な.so）
lib_filenames = ['vl53l0x_python' + suffix, 'vl53l0x_python.so']

# 探すディレクトリの候補（カレントディレクトリを追加）
_POSSIBLE_LIBRARY_LOCATIONS = ['.', '../bin'] + site.getsitepackages() + [site.getusersitepackages()]

_TOF_LIBRARY = None
for lib_location in _POSSIBLE_LIBRARY_LOCATIONS:
    for filename in lib_filenames:
        try:
            path = os.path.join(lib_location, filename)
            # ファイルが存在するか確認してからロードを試みる
            if os.path.exists(path):
                _TOF_LIBRARY = CDLL(path)
                # print(f"Loaded VL53L0X library from: {path}")
                break
        except OSError:
            pass
    if _TOF_LIBRARY is not None:
        break

if _TOF_LIBRARY is None:
    print("Search paths:", _POSSIBLE_LIBRARY_LOCATIONS)
    raise OSError('Could not find vl53l0x_python shared library. Please make sure you have compiled the C library.')


class VL53L0X:
    """VL53L0X ToF."""
    def __init__(self, i2c_bus=1, i2c_address=0x29, tca9548a_num=255, tca9548a_addr=0):
        """Initialize the VL53L0X ToF Sensor from ST"""
        self._i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self._tca9548a_num = tca9548a_num
        self._tca9548a_addr = tca9548a_addr
        self._dev = None
        self._h = None
        # Register Address
        self.ADDR_UNIT_ID_HIGH = 0x16 # Serial number high byte
        self.ADDR_UNIT_ID_LOW = 0x17 # Serial number low byte
        self.ADDR_I2C_ID_HIGH = 0x18 # Write serial number high byte for I2C address unlock
        self.ADDR_I2C_ID_LOW = 0x19 # Write serial number low byte for I2C address unlock
        self.ADDR_I2C_SEC_ADDR = 0x8a # Write new I2C address after unlock

    def open(self):
        # ここで一度だけ I2C デバイスを開く
        try:
            self._h = lgpio.i2c_open(self._i2c_bus, self.i2c_address)
        except lgpio.error as e:
            print(f"Error opening I2C bus {self._i2c_bus}: {e}")
            raise

        self._configure_i2c_library_functions()
        # Cライブラリの初期化
        self._dev = _TOF_LIBRARY.initialise(self.i2c_address, self._tca9548a_num, self._tca9548a_addr)

    def close(self):
        if self._h is not None:
            lgpio.i2c_close(self._h)
            self._h = None
        self._dev = None

    def _configure_i2c_library_functions(self):
        # --- I2C Read Callback ---
        def _i2c_read(address, reg, data_p, length):
            ret_val = 0
            
            # 現在のハンドルを使用する
            handle = self._h
            
            # もしライブラリが指定してきたアドレスが、現在開いているアドレスと違う場合
            # (アドレス変更処理中など)、一時的にそのアドレスで開く
            temp_handle = None
            if address != self.i2c_address:
                try:
                    temp_handle = lgpio.i2c_open(self._i2c_bus, address)
                    handle = temp_handle
                except Exception as e:
                    print(f"I2C Read Open Error (Addr {address:02x}): {e}")
                    return -1

            try:
                # 読み込み実行
                (count, result) = lgpio.i2c_read_i2c_block_data(handle, reg, length)
                
                # C側のポインタバッファにデータをコピー
                # resultはbytesオブジェクトなのでインデックスアクセスでintが取れる
                for index in range(length):
                    if index < count:
                        data_p[index] = result[index]
                    else:
                        data_p[index] = 0 # データが足りない場合のパディング
            except Exception as e:
                # エラー詳細を表示するとデバッグしやすい
                print(f"I2C Read Error (Reg {reg:02x}): {e}")
                ret_val = -1
            
            # 一時的に開いたハンドルなら閉じる
            if temp_handle is not None:
                lgpio.i2c_close(temp_handle)

            return ret_val

        # --- I2C Write Callback ---
        def _i2c_write(address, reg, data_p, length):
            ret_val = 0
            
            handle = self._h
            temp_handle = None
            if address != self.i2c_address:
                try:
                    temp_handle = lgpio.i2c_open(self._i2c_bus, address)
                    handle = temp_handle
                except Exception:
                    return -1

            # 書き込むデータをリストに変換
            data = []
            for index in range(length):
                data.append(data_p[index])
            
            try:
                # 書き込み実行
                lgpio.i2c_write_i2c_block_data(handle, reg, data)
            except Exception as e:
                print(f"I2C Write Error (Reg {reg:02x}): {e}")
                ret_val = -1

            if temp_handle is not None:
                lgpio.i2c_close(temp_handle)

            return ret_val

        # Pass i2c read/write function pointers to VL53L0X library.
        self._i2c_read_func = _I2C_READ_FUNC(_i2c_read)
        self._i2c_write_func = _I2C_WRITE_FUNC(_i2c_write)
        _TOF_LIBRARY.VL53L0X_set_i2c(self._i2c_read_func, self._i2c_write_func)

    def start_ranging(self, mode=Vl53l0xAccuracyMode.GOOD):
        """Start VL53L0X ToF Sensor Ranging"""
        _TOF_LIBRARY.startRanging(self._dev, mode)

    def stop_ranging(self):
        """Stop VL53L0X ToF Sensor Ranging"""
        _TOF_LIBRARY.stopRanging(self._dev)

    def get_distance(self):
        """Get distance from VL53L0X ToF Sensor"""
        return _TOF_LIBRARY.getDistance(self._dev)

    # This function included to show how to access the ST library directly
    # from python instead of through the simplified interface
    def get_timing(self):
        budget = c_uint(0)
        budget_p = pointer(budget)
        status = _TOF_LIBRARY.VL53L0X_GetMeasurementTimingBudgetMicroSeconds(self._dev, budget_p)
        if status == 0:
            return budget.value + 1000
        else:
            return 0

    def configure_gpio_interrupt(
            self, proximity_alarm_type=Vl53l0xGpioAlarmType.THRESHOLD_CROSSED_LOW,
            interrupt_polarity=Vl53l0xInterruptPolarity.HIGH, threshold_low_mm=250, threshold_high_mm=500):
        """
        Configures a GPIO interrupt from device, be sure to call "clear_interrupt" after interrupt is processed.
        """
        pin = c_uint8(0)  # 0 is only GPIO pin.
        device_mode = c_uint8(Vl53l0xDeviceMode.CONTINUOUS_RANGING)
        functionality = c_uint8(proximity_alarm_type)
        polarity = c_uint8(interrupt_polarity)
        status = _TOF_LIBRARY.VL53L0X_SetGpioConfig(self._dev, pin, device_mode, functionality, polarity)
        if status != 0:
            raise Vl53l0xError('Error setting VL53L0X GPIO config')

        threshold_low = c_uint32(threshold_low_mm << 16)
        threshold_high = c_uint32(threshold_high_mm << 16)
        status = _TOF_LIBRARY.VL53L0X_SetInterruptThresholds(self._dev, device_mode, threshold_low, threshold_high)
        if status != 0:
            raise Vl53l0xError('Error setting VL53L0X thresholds')

        # Ensure any pending interrupts are cleared.
        self.clear_interrupt()

    def clear_interrupt(self):
        mask = c_uint32(0)
        status = _TOF_LIBRARY.VL53L0X_ClearInterruptMask(self._dev, mask)
        if status != 0:
            raise Vl53l0xError('Error clearing VL53L0X interrupt')

    def change_address(self, new_address):
        if self._dev is not None:
            raise Vl53l0xError('Error changing VL53L0X address')
        
        # ハンドルがなければ一時的に開く
        opened_here = False
        if self._h is None:
            try:
                self._h = lgpio.i2c_open(self._i2c_bus, self.i2c_address)
                opened_here = True
            except lgpio.error:
                return

        if new_address != None and new_address != self.i2c_address:
            # read value from 0x16,0x17
            high = lgpio.i2c_read_byte_data(self._h, self.ADDR_UNIT_ID_HIGH)
            low = lgpio.i2c_read_byte_data(self._h, self.ADDR_UNIT_ID_LOW)

            # write value to 0x18,0x19
            lgpio.i2c_write_byte_data(self._h, self.ADDR_I2C_ID_HIGH, high)
            lgpio.i2c_write_byte_data(self._h, self.ADDR_I2C_ID_LOW, low)

            # write new_address to 0x1a
            lgpio.i2c_write_byte_data(self._h, self.ADDR_I2C_SEC_ADDR, new_address)

            self.i2c_address = new_address

        # ここで開いたなら閉じる
        if opened_here:
            lgpio.i2c_close(self._h)
            self._h = None
def main():
    tof = VL53L0X(i2c_bus=1,i2c_address=0x29)
    # I2C Address can change before tof.open()
    # tof.change_address(0x32)
    tof.open()
    # Start ranging
    tof.start_ranging(Vl53l0xAccuracyMode.BETTER)

    timing = tof.get_timing()
    if timing < 20000:
        timing = 20000
    print("Timing %d ms" % (timing/1000))

    for count in range(1, 101):
        distance = tof.get_distance()
        if distance > 0:
            print("%d mm, %d cm, %d" % (distance, (distance/10), count))

        time.sleep(timing/1000000.00)

    tof.stop_ranging()
    tof.close()
if __name__ == "__main__":
    main()