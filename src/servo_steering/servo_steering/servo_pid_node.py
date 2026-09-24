#!/usr/bin/env python3
"""
servo_pid_node.py — controle de direção em malha fechada (esterço pelo servo).

    /cmd_vel      angular.z (rad/s)           -> r  (giro desejado, vindo do Nav2)
    /bno055/imu   angular_velocity.z (rad/s)  -> y  (giro medido, filtrado)
    /wheel_odom   twist.linear.x (m/s)        -> v  (só para saber se o carro anda e em que sentido)

    e   = r - y
    u   = ff_gain*r + PID(e)                     (rad/s, mesma lógica do pid_controller original)
    ang = steering_sign * sentido(v) * degrees(u) -> servo (lgpio)

Por que o feedback é o giroscópio e não a "posição do servo":
um servo de hobby já tem um controle de posição interno e não informa onde está.
O que dá para medir (e o que o Nav2 realmente pede com angular.z) é quanto o
carrinho está girando. Então o PID corrige o ângulo do servo até o giro medido
bater com o giro pedido — igual à malha do servo do primeiro código.

Frequência do laço: 100 Hz.
Não rode o servo_node antigo junto com este nó (disputam o GPIO 18).
"""

import math
import time

import lgpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray

# ── Cole aqui os valores do servo_calibration.txt ────────────────────────────
SERVO_MIN_US    = 2000
SERVO_MAX_US    = 4010
SERVO_CENTER_US = 3000
ANGLE_MAX_DEG   = 29
ANGLE_MIN_DEG   = -29
# ─────────────────────────────────────────────────────────────────────────────

GPIO_CHIP = 4
GPIO_PIN  = 18
PWM_FREQ  = 100


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def angle_to_us(angle_deg: float) -> float:
    """Mesma conversão do servo_node original (valores da calibração)."""
    angle_deg = clamp(angle_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG)
    if angle_deg < 0:
        return SERVO_CENTER_US + (angle_deg / ANGLE_MIN_DEG) * (SERVO_MIN_US - SERVO_CENTER_US)
    return SERVO_CENTER_US + (angle_deg / ANGLE_MAX_DEG) * (SERVO_MAX_US - SERVO_CENTER_US)


class LowPassFilter:
    """Filtro passa-baixa de 1ª ordem para o giro cru do BNO055."""

    def __init__(self, alpha):
        self.alpha = alpha
        self.y = None

    def update(self, x):
        if self.y is None:
            self.y = x
        else:
            self.y += self.alpha * (x - self.y)
        return self.y


class PID:
    """
    Mesmo PID do motor_pid_node:
      - P: aproxima o giro medido do pedido
      - I: zera o erro em regime (ex.: servo desalinhado, carro puxando para um lado)
      - D: sobre a medição e filtrado, amortece a oscilação da direção
      - correção limitada (o PID nunca se afasta demais do que o Nav2 pediu)
      - anti-windup por integração condicional
    """

    def __init__(self, kp, ki, kd, corr_limit, out_min, out_max, d_tau=0.02):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.corr_limit = corr_limit
        self.out_min, self.out_max = out_min, out_max
        self.d_tau = d_tau
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.d_filt = 0.0
        self.prev_meas = None
        self.p = self.i = self.d = 0.0

    def update(self, setpoint, measurement, dt, u_ff=0.0):
        error = setpoint - measurement

        # ---- P ----
        self.p = self.kp * error

        # ---- D (sobre a medição, filtrado) ----
        d_raw = 0.0 if self.prev_meas is None else -(measurement - self.prev_meas) / dt
        self.prev_meas = measurement
        alpha = dt / (self.d_tau + dt)
        self.d_filt += alpha * (d_raw - self.d_filt)
        self.d = self.kd * self.d_filt

        # ---- I com anti-windup ----
        integral_try = self.integral + error * dt
        corr_try = self.p + self.ki * integral_try + self.d
        total_try = u_ff + corr_try
        sat_alto = (corr_try > self.corr_limit or total_try > self.out_max) and error > 0
        sat_baixo = (corr_try < -self.corr_limit or total_try < self.out_min) and error < 0
        if not (sat_alto or sat_baixo):
            self.integral = integral_try
        self.i = self.ki * self.integral

        corr = clamp(self.p + self.i + self.d, -self.corr_limit, self.corr_limit)
        return clamp(u_ff + corr, self.out_min, self.out_max), error


class ServoPidNode(Node):
    def __init__(self):
        super().__init__('servo_pid_node')

        # ---- parâmetros gerais ----
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)      # sem /cmd_vel -> centraliza
        self.declare_parameter('feedback_stale_sec', 0.1)   # sem IMU/odom -> malha aberta
        self.declare_parameter('min_speed_for_pid', 0.05)   # m/s; parado o giro é 0 sempre
        self.declare_parameter('gyro_filter_alpha', 0.2)
        self.declare_parameter('us_deadband', 2.0)          # não reenvia PWM por variação menor

        # ---- ganhos de EXEMPLO (erro em rad/s -> correção em rad/s) ----
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.2)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('d_filter_tau', 0.02)        # s
        self.declare_parameter('correction_limit', 0.4)     # rad/s, máximo que o PID soma
        self.declare_parameter('ff_gain', 1.0)              # 0.0 = PID puro

        # ---- sinais (confira no primeiro teste!) ----
        self.declare_parameter('steering_sign', -1.0)       # o pid_controller antigo invertia
        self.declare_parameter('reverse_compensation', True)

        gp = lambda n: self.get_parameter(n).value
        self.cmd_timeout = gp('cmd_timeout_sec')
        self.feedback_stale = gp('feedback_stale_sec')
        self.min_speed = gp('min_speed_for_pid')
        self.us_deadband = gp('us_deadband')
        self.ff_gain = gp('ff_gain')
        self.steering_sign = gp('steering_sign')
        self.reverse_comp = gp('reverse_compensation')

        # saída em "rad/s" que, pela conversão degrees(), vira no máximo ±ANGLE_MAX_DEG
        out_lim = math.radians(max(abs(ANGLE_MIN_DEG), abs(ANGLE_MAX_DEG)))
        self.pid = PID(gp('kp'), gp('ki'), gp('kd'),
                       corr_limit=gp('correction_limit'),
                       out_min=-out_lim, out_max=out_lim,
                       d_tau=gp('d_filter_tau'))
        self.out_lim = out_lim
        self.gyro_filter = LowPassFilter(gp('gyro_filter_alpha'))

        # ---- estado ----
        self.setpoint = 0.0       # angular.z desejado
        self.cmd_linear = 0.0     # linear.x desejado (sentido de marcha)
        self.yaw_rate = 0.0       # giro medido filtrado
        self.v_medido = 0.0
        self.last_cmd_time = None
        self.last_imu_time = None
        self.last_odom_time = None
        self.last_loop_time = None
        self._sentido_anterior = 1.0
        self._last_us = None

        # ---- hardware ----
        self._h = lgpio.gpiochip_open(GPIO_CHIP)
        lgpio.gpio_claim_output(self._h, GPIO_PIN)
        self._move(0.0)

        # ---- ROS ----
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Imu, '/bno055/imu', self.imu_cb, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.wheel_odom_cb, 10)
        # [setpoint, medido, erro, P, I, D, u (rad/s), ângulo aplicado (graus)]
        self.pub_debug = self.create_publisher(Float64MultiArray, '/servo_pid/debug', 10)

        rate = gp('control_rate_hz')
        self.dt_nominal = 1.0 / rate
        self.create_timer(self.dt_nominal, self.control_loop)

        self.get_logger().info(
            f'servo_pid_node @ {rate:.0f} Hz | Kp={gp("kp")} Ki={gp("ki")} Kd={gp("kd")} '
            f'ff={self.ff_gain} steering_sign={self.steering_sign}'
        )

    # ─────────────────────────── callbacks ───────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        self.setpoint = msg.angular.z
        self.cmd_linear = msg.linear.x
        self.last_cmd_time = self.get_clock().now()

    def imu_cb(self, msg: Imu):
        self.yaw_rate = self.gyro_filter.update(msg.angular_velocity.z)
        self.last_imu_time = self.get_clock().now()

    def wheel_odom_cb(self, msg: Odometry):
        self.v_medido = msg.twist.twist.linear.x
        self.last_odom_time = self.get_clock().now()

    # ─────────────────────────── helpers ───────────────────────────
    def _seconds_since(self, t):
        if t is None:
            return float('inf')
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def _dt_real(self):
        agora = self.get_clock().now()
        if self.last_loop_time is None:
            dt = self.dt_nominal
        else:
            dt = (agora - self.last_loop_time).nanoseconds / 1e9
            if not (0.2 * self.dt_nominal < dt < 5.0 * self.dt_nominal):
                dt = self.dt_nominal
        self.last_loop_time = agora
        return dt

    # ─────────────────────────── laço de controle ───────────────────────────
    def control_loop(self):
        dt = self._dt_real()

        # (1) watchdog do setpoint: sem /cmd_vel -> direção centralizada
        if self._seconds_since(self.last_cmd_time) > self.cmd_timeout:
            self.setpoint = 0.0
            self.cmd_linear = 0.0

        imu_ok = self._seconds_since(self.last_imu_time) <= self.feedback_stale
        odom_ok = self._seconds_since(self.last_odom_time) <= self.feedback_stale

        # O carro só gira se estiver andando. Parado, o giro medido é 0 qualquer
        # que seja o esterço, e a integral jogaria o servo para o batente.
        v = self.v_medido if odom_ok else self.cmd_linear
        andando = abs(v) >= self.min_speed

        # Em ré, o mesmo esterço gira o carro para o lado oposto (ω = v·tanδ/L),
        # então o sinal do comando precisa inverter, senão a realimentação vira positiva.
        sentido = -1.0 if (self.reverse_comp and self.cmd_linear < 0) else 1.0
        if sentido != self._sentido_anterior:
            self.pid.reset()
            self._sentido_anterior = sentido

        u_ff = self.ff_gain * self.setpoint

        if imu_ok and andando:
            u, erro = self.pid.update(self.setpoint, self.yaw_rate, dt, u_ff)
        else:
            # malha aberta: só o feedforward (comportamento do servo_node original)
            self.pid.reset()
            u = clamp(u_ff, -self.out_lim, self.out_lim)
            erro = self.setpoint - self.yaw_rate
            if not imu_ok:
                self.get_logger().warn('/bno055/imu obsoleta/ausente — servo sem correção PID',
                                       throttle_duration_sec=2.0)

        angle_deg = self.steering_sign * sentido * math.degrees(u)
        angle_aplicado = self._move(angle_deg)
        self._publicar(u, erro, angle_aplicado)

    # ─────────────────────────── saída ───────────────────────────
    def _publicar(self, u, erro, angle_deg):
        dbg = Float64MultiArray()
        dbg.data = [float(self.setpoint), float(self.yaw_rate), float(erro),
                    self.pid.p, self.pid.i, self.pid.d, float(u), float(angle_deg)]
        self.pub_debug.publish(dbg)
        self.get_logger().debug(
            f'r={self.setpoint:+.3f} y={self.yaw_rate:+.3f} e={erro:+.3f} ang={angle_deg:+.1f}°')

    # ─────────────────────────── hardware ───────────────────────────
    def _move(self, angle_deg: float) -> float:
        """Envia o ângulo ao servo e retorna o ângulo efetivamente aplicado (já limitado)."""
        angle_deg = clamp(angle_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG)
        us = angle_to_us(angle_deg)
        # reenviar PWM idêntico a 100 Hz só gera tremedeira no servo
        if self._last_us is not None and abs(us - self._last_us) < self.us_deadband:
            return angle_deg
        duty = (us / 20000.0) * 100  # mesma fórmula do servo_node original (bate com a calibração)
        lgpio.tx_pwm(self._h, GPIO_PIN, PWM_FREQ, duty)
        self._last_us = us
        return angle_deg

    def destroy_node(self):
        self._last_us = None
        self._move(0.0)
        time.sleep(0.3)
        lgpio.tx_pwm(self._h, GPIO_PIN, 0, 0)
        lgpio.gpiochip_close(self._h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()