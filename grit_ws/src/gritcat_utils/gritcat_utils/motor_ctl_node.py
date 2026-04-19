import sys
import time
import lgpio

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ==========================================
# 提供いただいたクラス (MotorDriver, GritMotor)
# ==========================================

class MotorDriver:
    """
    Raspberry Piでlgpioライブラリを使用して、2つのDCモーターを
    PWM制御するためのモータードライバクラスです。
    """
    def __init__(self, h, pins={
                                'ENA': 20,  # 左モーター(motor1) PWM
                                'IN1': 16,  # 左モーター(motor1)
                                'IN2': 19,  # 左モーター(motor1)
                                'ENB': 21,  # 右モーター(motor2) PWM
                                'IN3': 6,   # 右モーター(motor2)
                                'IN4': 12,  # 右モーター(motor2)
                                }, batt_v=14.4, abs_v=12.0):
        self.pins = pins
        self.pwm_freq = 100  # PWM周波数 (Hz)
        self.max_duty = 100 * min(abs_v/batt_v, 1)
        try:
            self.h = h
        except lgpio.error as e:
            print(f"GPIOチップのオープンに失敗しました: {e}")
            sys.exit(1)

        for pin in self.pins.values():
            lgpio.gpio_claim_output(self.h, pin)
            lgpio.gpio_write(self.h, pin, 0)

        lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, 0)

    def motor(self, left_motor_speed, right_motor_speed):
        # --- 左モーター (motor1) の制御 ---
        if left_motor_speed > 0:
            lgpio.gpio_write(self.h, self.pins['IN1'], 1)
            lgpio.gpio_write(self.h, self.pins['IN2'], 0)
            speed = min(left_motor_speed, self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, int(speed))
        elif left_motor_speed < 0:
            lgpio.gpio_write(self.h, self.pins['IN1'], 0)
            lgpio.gpio_write(self.h, self.pins['IN2'], 1)
            speed = min(abs(left_motor_speed), self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, int(speed))
        else:
            lgpio.gpio_write(self.h, self.pins['IN1'], 0)
            lgpio.gpio_write(self.h, self.pins['IN2'], 0)
            lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, 0)

        # --- 右モーター (motor2) の制御 ---
        if right_motor_speed > 0:
            lgpio.gpio_write(self.h, self.pins['IN3'], 1)
            lgpio.gpio_write(self.h, self.pins['IN4'], 0)
            speed = min(right_motor_speed, self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, int(speed))
        elif right_motor_speed < 0:
            lgpio.gpio_write(self.h, self.pins['IN3'], 0)
            lgpio.gpio_write(self.h, self.pins['IN4'], 1)
            speed = min(abs(right_motor_speed), self.max_duty)
            lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, int(speed))
        else:
            lgpio.gpio_write(self.h, self.pins['IN3'], 0)
            lgpio.gpio_write(self.h, self.pins['IN4'], 0)
            lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, 0)

    def cleanup(self):
        print("クリーンアップ処理を実行します...")
        self.motor(0, 0)
        lgpio.tx_pwm(self.h, self.pins['ENA'], self.pwm_freq, 0)
        lgpio.tx_pwm(self.h, self.pins['ENB'], self.pwm_freq, 0)
        
    
class GritMotor:
    def __init__(self,h):
        self.h=h
        self.leftMD=MotorDriver(h = self.h,
                                pins = {'ENA': 18,  # 左モーター(motor1) PWM
                                        'IN1': 10,  # 左モーター(motor1)
                                        'IN2': 25,  # 左モーター(motor1)
                                        'ENB': 12,  # 左モーター(motor2) PWM
                                        'IN3': 7,  # 左モーター(motor2)
                                        'IN4': 11,   # 左モーター(motor2)
                                        },
                                batt_v=16.6,
                                abs_v=12.0)
        self.rightMD=MotorDriver(h = self.h,
                                pins = {'ENA': 13,  # 右モーター(motor1) PWM
                                        'IN1': 5,  # 右モーター(motor1)
                                        'IN2': 6,  # 右モーター(motor1)
                                        'ENB': 19,  # 右モーター(motor2) PWM
                                        'IN3': 1,  # 右モーター(motor2)
                                        'IN4': 0,   # 右モーター(motor2)
                                        },
                                batt_v=16.6,
                                abs_v=12.0)
    def move(self, left_motor_speed, right_motor_speed):
        self.leftMD.motor(left_motor_speed,left_motor_speed)
        self.rightMD.motor(right_motor_speed,right_motor_speed)
    
    def cleanup(self):
        self.leftMD.cleanup()
        self.rightMD.cleanup()
# ==========================================
# ROS 2 ノードの実装
# ==========================================

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # --- GPIOとモーターの初期化 ---
        try:
            self.h = lgpio.gpiochip_open(0)
            # 使用例にあった追加のピン設定
            lgpio.gpio_claim_output(self.h, 22)
            lgpio.gpio_write(self.h, 22, 1)
            
            self.motor = GritMotor(self.h)
            self.get_logger().info('MotorDriver Initialized successfully.')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO/Motor: {e}")
            sys.exit(1)

        # --- サブスクライバの作成 ---
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # --- 変換係数 (実機に合わせて調整してください) ---
        # Twistメッセージ (m/s, rad/s) の数値を -100〜100 のPWMパーセンテージに変換するための係数
        self.linear_coef = 100.0   # 例: linear.x = 1.0 m/s のときPWM 100%
        self.angular_coef = 50.0   # 例: angular.z = 1.0 rad/s のときPWM 50% を増減させる
        
        # --- 安全装置 (フェイルセーフ用タイマー) ---
        self.timeout_sec = 0.5  # コマンドが0.5秒間途絶えたら自動停止
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        
        self.get_logger().info('Motor Control Node has been started.')

    def cmd_vel_callback(self, msg):
        """
        cmd_velを受信した際のコールバック関数
        """
        # 最後にコマンドを受け取った時間を更新
        self.last_cmd_time = self.get_clock().now()
        
        # Twistメッセージの取得
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # 差動駆動モデルをベースに左右の速度を計算
        # 前進(linear_x > 0)の場合、左右ともにプラス
        # 左旋回(angular_z > 0)の場合、左がマイナス（または遅く）、右がプラス（または速く）
        raw_left_speed = (linear_x * self.linear_coef) - (angular_z * self.angular_coef)
        raw_right_speed = (linear_x * self.linear_coef) + (angular_z * self.angular_coef)

        # 速度を -100 〜 100 の範囲にクリップ（制限）する
        left_speed = max(min(raw_left_speed, 100.0), -100.0)
        right_speed = max(min(raw_right_speed, 100.0), -100.0)

        # モーターを駆動
        self.motor.move(left_speed, right_speed)

    def check_timeout(self):
        """
        コマンドが一定時間来ていないかチェックし、来ていなければ停止する
        """
        now = self.get_clock().now()
        elapsed_time = (now - self.last_cmd_time).nanoseconds / 1e9
        
        if elapsed_time > self.timeout_sec:
            # タイムアウトした場合はモーターを停止する
            self.motor.move(0.0, 0.0)

    def destroy_node(self):
        """
        ノード終了時の処理 (Ctrl+Cなどで呼ばれます)
        """
        self.get_logger().info('Shutting down... Stopping motors and cleaning up GPIO.')
        if hasattr(self, 'motor'):
            self.motor.cleanup()
        if hasattr(self, 'h'):
            lgpio.gpio_write(self.h, 22, 0) # 確保したピンをOFF
            lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = MotorControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT) received.')
    finally:
        # シャットダウン処理
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()