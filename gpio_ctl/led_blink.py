import time
import lgpio

# 使用するGPIOピン番号
LED_PIN = 23

# GPIOチップを開く
h = lgpio.gpiochip_open(0)

# GPIOピンを出力として設定
lgpio.gpio_claim_output(h, LED_PIN)

try:
    while True:
        # LEDをオンにする
        lgpio.gpio_write(h, LED_PIN, 1)
        time.sleep(1)

        # LEDをオフにする
        lgpio.gpio_write(h, LED_PIN, 0)
        time.sleep(1)

except KeyboardInterrupt:
    # プログラム終了時にLEDをオフにする
    lgpio.gpio_write(h, LED_PIN, 0)
    # GPIOチップを閉じる
    lgpio.gpiochip_close(h)