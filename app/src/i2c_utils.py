import smbus2

def scan_i2c_bus(bus_num=1):
    """
    I2Cバスをスキャンして、応答のあるデバイスのアドレスと推定されるデバイス名を返します。
    Returns:
        list of dict: [{'addr': '0x20', 'name': 'MCP23017'}, ...]
    """
    
    # このロボットで使われている可能性のあるデバイスのマッピング
    KNOWN_DEVICES = {
        0x20: "MCP23017 (IO Expander)",
        0x28: "BNO055 (IMU)",
        0x29: "VL53L0X (Distance)",
        0x3C: "SSD1306 (OLED)",
        0x40: "PCA9685 (Servo Driver)",
        0x68: "MPU6050 (IMU) or DS3231",
        0x76: "BME280 (Environment)",
        0x77: "BME280 (Environment - Alt Addr)"
    }

    found_devices = []
    
    try:
        with smbus2.SMBus(bus_num) as bus:
            # 一般的なアドレス範囲 (0x03 - 0x77) をスキャン
            for addr in range(0x03, 0x78):
                try:
                    # write_quick はデバイスに影響を与えずに存在確認する最も軽い方法
                    bus.write_quick(addr)
                    
                    # 応答があればリストに追加
                    device_name = KNOWN_DEVICES.get(addr, "Unknown Device")
                    found_devices.append({
                        "address": hex(addr),
                        "name": device_name
                    })
                except OSError:
                    # 応答なし (該当アドレスにデバイスがいない)
                    pass
    except Exception as e:
        print(f"I2C Scan Error: {e}")
        return []

    return found_devices
