import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import lgpio

# ── Pinos GPIO ──────────────────────────────────────────
RPWM      = 18
LPWM      = 19
R_EN      = 23
L_EN      = 24
SERVO_PIN = 12

# ── Parâmetros do motor ─────────────────────────────────
FREQ_MOTOR      = 1000   # Hz
FREQ_SERVO      = 50     # Hz
SERVO_NEUTRO    = 7.5
SERVO_MAX_ESQ   = 10.0
SERVO_MAX_DIR   = 5.0
MAX_VEL_LINEAR  = 1.0
MAX_VEL_ANGULAR = 1.0

V_BATERIA   = 11.1
V_MOTOR_MAX = 7.0
DUTY_MAX    = min((V_MOTOR_MAX / V_BATERIA) * 100, 100.0)


class MotorControl(Node):
    def __init__(self):
        super().__init__('motor_control')

        # Abre o chip GPIO do RPi5
        self.h = lgpio.gpiochip_open(0)

        # Declara todos os pinos como saída — obrigatório antes de usar PWM
        lgpio.gpio_claim_output(self.h, RPWM)
        lgpio.gpio_claim_output(self.h, LPWM)
        lgpio.gpio_claim_output(self.h, R_EN)
        lgpio.gpio_claim_output(self.h, L_EN)
        lgpio.gpio_claim_output(self.h, SERVO_PIN)

        # Habilita a ponte H
        lgpio.gpio_write(self.h, R_EN, 1)
        lgpio.gpio_write(self.h, L_EN, 1)

        # Inicia PWM do motor com duty 0
        lgpio.tx_pwm(self.h, RPWM, FREQ_MOTOR, 0)
        lgpio.tx_pwm(self.h, LPWM, FREQ_MOTOR, 0)

        # Inicia PWM do servo na posição neutra
        lgpio.tx_pwm(self.h, SERVO_PIN, FREQ_SERVO, SERVO_NEUTRO)

        # Assina o /cmd_vel
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info(
            f'Motor control iniciado! | Duty máximo: {DUTY_MAX:.1f}%')

    def cmd_vel_callback(self, msg: Twist):
        try:
            self._set_motor(msg.linear.x)
            self._set_servo(msg.angular.z)
        except Exception as e:
            self.get_logger().error(f'Erro no callback: {e}')

    def _set_motor(self, linear: float):
        duty = min(abs(linear) / MAX_VEL_LINEAR * 100, DUTY_MAX)

        if linear > 0:
            lgpio.tx_pwm(self.h, RPWM, FREQ_MOTOR, duty)
            lgpio.tx_pwm(self.h, LPWM, FREQ_MOTOR, 0)
            self.get_logger().debug(f'Frente — duty: {duty:.1f}%')
        elif linear < 0:
            lgpio.tx_pwm(self.h, RPWM, FREQ_MOTOR, 0)
            lgpio.tx_pwm(self.h, LPWM, FREQ_MOTOR, duty)
            self.get_logger().debug(f'Ré — duty: {duty:.1f}%')
        else:
            lgpio.tx_pwm(self.h, RPWM, FREQ_MOTOR, 0)
            lgpio.tx_pwm(self.h, LPWM, FREQ_MOTOR, 0)

    def _set_servo(self, angular: float):
        ratio = max(-1.0, min(1.0, angular / MAX_VEL_ANGULAR))
        if ratio >= 0:
            duty_servo = SERVO_NEUTRO + (SERVO_MAX_ESQ - SERVO_NEUTRO) * ratio
        else:
            duty_servo = SERVO_NEUTRO + (SERVO_NEUTRO - SERVO_MAX_DIR) * ratio
        lgpio.tx_pwm(self.h, SERVO_PIN, FREQ_SERVO, duty_servo)

    def destroy_node(self):
        # Para tudo e libera os pinos ao encerrar
        lgpio.tx_pwm(self.h, RPWM,      FREQ_MOTOR, 0)
        lgpio.tx_pwm(self.h, LPWM,      FREQ_MOTOR, 0)
        lgpio.tx_pwm(self.h, SERVO_PIN, FREQ_SERVO, SERVO_NEUTRO)
        lgpio.gpiochip_close(self.h)
        super().destroy_node()


def main():
    rclpy.init()
    node = MotorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()