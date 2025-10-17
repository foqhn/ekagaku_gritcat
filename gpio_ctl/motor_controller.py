import lgpio
import time
import sys

class MotorDriver:
    """
    Raspberry Piでlgpioライブラリを使用して、2つのDCモーターを
    PWM制御するためのモータードライバクラスです。
    """
    def __init__(self, h ,pins={
                                        'ENA': 20,  # 左モーター(motor1) PWM
                                        'IN1': 16,  # 左モーター(motor1)
                                        'IN2': 19,  # 左モーター(motor1)
                                        'ENB': 21,  # 右モーター(motor2) PWM
                                        'IN3': 6,  # 右モーター(motor2)
                                        'IN4': 12,   # 右モーター(motor2)
                                        },batt_v=14.4,abs_v=12.0):
        """
        GPIOピンの初期設定とPWMのセットアップを行います。
        gpiochip: 使用するGPIOチップの番号 (Raspberry Pi 4では通常0)
        """
        self.pins = pins
        self.pwm_freq = 100  # PWM周波数 (Hz)
        self.max_duty=100*min(abs_v/batt_v,1)
        try:
            # GPIOチップを開く
            self.h = h
        except lgpio.error as e:
            print(f"GPIOチップのオープンに失敗しました: {e}")
            print("lgpioデーモンが実行されているか、または十分な権限があるか確認してください。")
            sys.exit(1)

        # 全てのピンを出力として確保(claim)する
        for pin in self.pins.values():
            lgpio.gpio_claim_output(self.h, pin)
            lgpio.gpio_write(self.h, pin, 0) # 初期状態をLOWに設定

        # PWMの初期設定 (周波数100Hz, デューティサイクル0%)
        # lgpioではPWM専用のオブジェクトはなく、tx_pwm関数で直接制御します
        lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, 0)

    def motor(self, left_motor_speed, right_motor_speed):
        """
        左右のモーターの速度と回転方向を制御します。
        speed: -100から100の範囲で速度を指定します。
               正の値で正転、負の値で逆転、0で停止します。
        """
        # --- 左モーター (motor1) の制御 ---
        if left_motor_speed > 0:
            # 正転
            lgpio.gpio_write(self.h, self.pins['IN1'], 1)
            lgpio.gpio_write(self.h, self.pins['IN2'], 0)
            # 速度が100を超えないように制御
            speed = min(left_motor_speed, self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, int(speed))
        elif left_motor_speed < 0:
            # 逆転
            lgpio.gpio_write(self.h, self.pins['IN1'], 0)
            lgpio.gpio_write(self.h, self.pins['IN2'], 1)
            # 速度が100を超えないように制御（絶対値を使用）
            speed = min(abs(left_motor_speed), self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, int(speed))
        else:
            # 停止
            lgpio.gpio_write(self.h, self.pins['IN1'], 0)
            lgpio.gpio_write(self.h, self.pins['IN2'], 0)
            lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, 0)

        # --- 右モーター (motor2) の制御 ---
        if right_motor_speed > 0:
            # 正転
            lgpio.gpio_write(self.h, self.pins['IN3'], 1)
            lgpio.gpio_write(self.h, self.pins['IN4'], 0)
            speed = min(right_motor_speed, self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, int(speed))
        elif right_motor_speed < 0:
            # 逆転
            lgpio.gpio_write(self.h, self.pins['IN3'], 0)
            lgpio.gpio_write(self.h, self.pins['IN4'], 1)
            speed = min(abs(right_motor_speed), self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, int(speed))
        else:
            # 停止
            lgpio.gpio_write(self.h, self.pins['IN3'], 0)
            lgpio.gpio_write(self.h, self.pins['IN4'], 0)
            lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, 0)

    def cleanup(self):
        """
        GPIOの設定をクリーンアップします。
        """
        print("クリーンアップ処理を実行します...")
        self.motor(0, 0) # モーターを停止
        # PWMを停止 (tx_pwmでデューティサイクルを0にしているので実質不要ですが念のため)
        lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, 0)
        
        # GPIOハンドルを閉じる (確保したピンは自動的に解放される)
        
    
class GritMotor:
    def __init__(self,h):
        self.h=h
        self.leftMD=MotorDriver(h = self.h,
                                pins = {'ENA': 12,  # 左モーター(motor1) PWM
                                        'IN1': 22,  # 左モーター(motor1)
                                        'IN2': 23,  # 左モーター(motor1)
                                        'ENB': 20,  # 右モーター(motor2) PWM
                                        'IN3': 26,  # 右モーター(motor2)
                                        'IN4': 27,   # 右モーター(motor2)
                                        },
                                batt_v=16.6,
                                abs_v=12.0)
        self.rightMD=MotorDriver(h = self.h,
                                pins = {'ENA': 13,  # 左モーター(motor1) PWM
                                        'IN1': 24,  # 左モーター(motor1)
                                        'IN2': 25,  # 左モーター(motor1)
                                        'ENB': 21,  # 右モーター(motor2) PWM
                                        'IN3': 4,  # 右モーター(motor2)
                                        'IN4': 5,   # 右モーター(motor2)
                                        },
                                batt_v=16.6,
                                abs_v=12.0)
    def move(self, left_motor_speed, right_motor_speed):
        self.leftMD.motor(left_motor_speed,left_motor_speed)
        self.rightMD.motor(right_motor_speed,right_motor_speed)
    
    def cleanup(self):
        self.leftMD.cleanup()
        self.rightMD.cleanup()
           

# ### このクラスの使用例
if __name__ == '__main__':
    h= lgpio.gpiochip_open(0)
    my_motor = None # finallyブロックで参照できるようにするため
    
    try:
        lgpio.gpio_claim_output(h, 17)
        # MotorDriverクラスのインスタンスを作成
        my_motor = GritMotor(h)
        
        lgpio.gpio_write(h, 17, 1)
        
        print("前進 (速度50%) 3秒間")
        my_motor.move(50, 50)
        time.sleep(3)

        print("後退 (速度50%) 3秒間")
        my_motor.move(-50, -50)
        time.sleep(3)

        print("その場で右回転 (左:正転80%, 右:逆転80%) 3秒間")
        my_motor.move(50, -50)
        time.sleep(3)

        print("左カーブ (左:30%, 右:70%) 3秒間")
        my_motor.move(-50, 50)
        time.sleep(3)

        print("停止")
        my_motor.move(0, 0)
        time.sleep(2)

        lgpio.gpio_write(h, 17, 0)

    except KeyboardInterrupt:
        print("\nプログラムを終了します。")
    finally:
        # プログラム終了時にGPIOをクリーンアップ
        if my_motor:
            my_motor.cleanup()
            lgpio.gpiochip_close(h)