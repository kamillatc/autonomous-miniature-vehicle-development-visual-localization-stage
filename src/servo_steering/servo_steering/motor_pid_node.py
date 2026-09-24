#!/usr/bin/env python3
"""
motor_pid_node — controle de velocidade do motor de tração em malha fechada.

    /cmd_vel     (Twist.linear.x, m/s)     -> r  (setpoint)
    /wheel_odom  (Odometry twist.linear.x) -> y  (velocidade medida no encoder)

    e = r - y
    u = feedforward(r) + Kp*e + Ki*∫e dt + Kd*d(-y)/dt      (u = duty PWM)

    u -> BTS7960 (RPWM / LPWM)
Frequência do laço e da publicação: 100 Hz.
Não rode o motor_node antigo junto com este nó (disputam os mesmos pinos GPIO).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from gpiozero import PWMOutputDevice, DigitalOutputDevice

# ── Pinos (BTS7960) ──────────────────────────────────────────────────────────
PIN_RPWM = 6    # Sentido Frente
PIN_LPWM = 13   # Sentido Trás
PIN_REN  = 2    # Enable Direito (R_EN)
PIN_LEN  = 3    # Enable Esquerdo (L_EN)

# ── Motor / alimentação ─────────────────────────────────────────────────────
PWM_FREQ    = 1000   # Hz
DEAD_TIME   = 0.05   # s — tempo com PWM zerado antes de inverter o sentido
V_BATERIA   = 11.1   # V  ← ajuste aqui
V_MAX_MOTOR = 6.0    # V
DUTY_MAX    = min(V_MAX_MOTOR / V_BATERIA, 1.0)   # proteção de tensão do motor

MAX_LINEAR_MS = 1.0  # m/s estimados com duty = DUTY_MAX (usado só no feedforward)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PID:
    """
    PID discreto:
      - P: aproxima a saída do setpoint (sozinho deixa erro residual)
      - I: elimina o erro em regime permanente
      - D: calculado sobre a MEDIÇÃO (não sobre o erro) e filtrado,
           amortece a oscilação sem dar "chute" quando o setpoint muda em degrau
      - anti-windup por integração condicional: a integral para de crescer
        enquanto a saída está saturada no mesmo sentido do erro
    """

    def __init__(self, kp, ki, kd, out_min, out_max, d_tau=0.02):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.d_tau = d_tau          # constante de tempo do filtro da derivada (s)
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

        # ---- D (sobre a medição, filtrado passa-baixa) ----
        if self.prev_meas is None:
            d_raw = 0.0
        else:
            d_raw = -(measurement - self.prev_meas) / dt
        self.prev_meas = measurement
        alpha = dt / (self.d_tau + dt)
        self.d_filt += alpha * (d_raw - self.d_filt)
        self.d = self.kd * self.d_filt

        # ---- I com anti-windup ----
        integral_try = self.integral + error * dt
        u_try = u_ff + self.p + self.ki * integral_try + self.d
        saturou_alto = u_try > self.out_max and error > 0
        saturou_baixo = u_try < self.out_min and error < 0
        if not (saturou_alto or saturou_baixo):
            self.integral = integral_try
        self.i = self.ki * self.integral

        u = u_ff + self.p + self.i + self.d
        return clamp(u, self.out_min, self.out_max), error


class MotorPidNode(Node):
    def __init__(self):
        super().__init__('motor_pid_node')

        # ---- parâmetros gerais ----
        self.declare_parameter('control_rate_hz', 100.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)      # sem /cmd_vel -> para
        self.declare_parameter('feedback_stale_sec', 0.1)   # sem /wheel_odom -> malha aberta
        self.declare_parameter('stop_threshold', 0.01)      # |setpoint| abaixo disso = parar

        # ---- ganhos de EXEMPLO (unidade: duty por m/s de erro) ----
        self.declare_parameter('kp', 0.4)
        self.declare_parameter('ki', 1.0)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('d_filter_tau', 0.02)        # s

        # feedforward: duty "chutado" a partir do setpoint; o PID só corrige a diferença.
        # Coloque 0.0 para testar PID puro.
        self.declare_parameter('ff_gain', DUTY_MAX / MAX_LINEAR_MS)  # duty por m/s
        self.declare_parameter('duty_max', DUTY_MAX)

        gp = lambda n: self.get_parameter(n).value
        self.cmd_timeout = gp('cmd_timeout_sec')
        self.feedback_stale = gp('feedback_stale_sec')
        self.stop_threshold = gp('stop_threshold')
        self.ff_gain = gp('ff_gain')
        self.duty_max = min(gp('duty_max'), DUTY_MAX)  # nunca passa do limite de tensão

        self.pid = PID(gp('kp'), gp('ki'), gp('kd'),
                       out_min=-self.duty_max, out_max=self.duty_max,
                       d_tau=gp('d_filter_tau'))

        # ---- estado ----
        self.setpoint = 0.0
        self.medido = 0.0
        self.last_cmd_time = None
        self.last_odom_time = None
        self.last_loop_time = None

        self._ultima_dir = 0            # último sentido efetivamente aplicado
        self._t_ultima_ativa = None     # instante em que esse sentido foi aplicado

        # ---- hardware ----
        self._rpwm = PWMOutputDevice(PIN_RPWM, frequency=PWM_FREQ)
        self._lpwm = PWMOutputDevice(PIN_LPWM, frequency=PWM_FREQ)
        self._r_en = DigitalOutputDevice(PIN_REN)
        self._l_en = DigitalOutputDevice(PIN_LEN)
        self._zerar_pwm()
        self._r_en.on()
        self._l_en.on()

        # ---- ROS ----
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.wheel_odom_cb, 10)
        # [setpoint, medido, erro, P, I, D, duty] — útil para sintonizar no PlotJuggler/rqt_plot
        self.pub_debug = self.create_publisher(Float64MultiArray, '/motor_pid/debug', 10)

        rate = gp('control_rate_hz')
        self.dt_nominal = 1.0 / rate
        self.create_timer(self.dt_nominal, self.control_loop)

        self.get_logger().info(
            f'motor_pid_node @ {rate:.0f} Hz | Kp={gp("kp")} Ki={gp("ki")} Kd={gp("kd")} '
            f'ff={self.ff_gain:.3f} duty_max={self.duty_max:.3f}'
        )

    # ─────────────────────────── callbacks ───────────────────────────
    def cmd_vel_cb(self, msg: Twist):
        novo = msg.linear.x
        # inversão de sentido pedida: a integral acumulada no sentido antigo só atrapalha
        if novo * self.setpoint < 0:
            self.pid.reset()
        self.setpoint = novo
        self.last_cmd_time = self.get_clock().now()

    def wheel_odom_cb(self, msg: Odometry):
        self.medido = msg.twist.twist.linear.x
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
            # protege contra dt absurdo (primeiro ciclo, travadas do SO)
            if not (0.2 * self.dt_nominal < dt < 5.0 * self.dt_nominal):
                dt = self.dt_nominal
        self.last_loop_time = agora
        return dt

    # ─────────────────────────── laço de controle ───────────────────────────
    def control_loop(self):
        dt = self._dt_real()

        # (1) watchdog do setpoint
        if self._seconds_since(self.last_cmd_time) > self.cmd_timeout:
            self.setpoint = 0.0

        # parada: zera tudo, sem deixar o PID "caçar" em torno de zero
        if abs(self.setpoint) < self.stop_threshold:
            self.pid.reset()
            duty_aplicado = self._aplicar_duty(0.0)
            self._publicar(duty_aplicado, erro=0.0 - self.medido)
            return

        u_ff = self.ff_gain * self.setpoint
        odom_ok = self._seconds_since(self.last_odom_time) <= self.feedback_stale

        if odom_ok:
            duty, erro = self.pid.update(self.setpoint, self.medido, dt, u_ff)
        else:
            # (2) sem feedback confiável: malha aberta só com feedforward
            self.pid.reset()
            duty = clamp(u_ff, -self.duty_max, self.duty_max)
            erro = float('nan')
            self.get_logger().warn('/wheel_odom obsoleto/ausente — malha aberta (só feedforward)',
                                   throttle_duration_sec=2.0)

        duty_aplicado = self._aplicar_duty(duty)
        self._publicar(duty_aplicado, erro)

    # ─────────────────────────── saída ───────────────────────────
    def _publicar(self, duty, erro):
        out = Twist()
        out.linear.x = float(duty)   # PWM com sinal: |duty| em 0..1
        self.pub_pwm.publish(out)

        dbg = Float64MultiArray()
        dbg.data = [float(self.setpoint), float(self.medido), float(erro),
                    self.pid.p, self.pid.i, self.pid.d, float(duty)]
        self.pub_debug.publish(dbg)

        self.get_logger().debug(
            f'r={self.setpoint:+.3f} y={self.medido:+.3f} e={erro:+.3f} duty={duty:+.3f}')

    # ─────────────────────────── hardware ───────────────────────────
    def _zerar_pwm(self):
        """Zera AMBOS os canais. Nunca ative RPWM e LPWM ao mesmo tempo."""
        self._rpwm.value = 0.0
        self._lpwm.value = 0.0

    def _aplicar_duty(self, duty: float) -> float:
        """
        Aplica o duty com sinal no BTS7960 e retorna o duty realmente aplicado.
        Dead-time NÃO bloqueante: na inversão de sentido o PWM fica zerado
        até passar DEAD_TIME desde a última vez que o sentido antigo esteve ativo
        (um sleep() aqui travaria o laço de 100 Hz).
        """
        nova_dir = 1 if duty > 0 else (-1 if duty < 0 else 0)

        if nova_dir == 0:
            self._zerar_pwm()
            return 0.0

        if (self._ultima_dir != 0 and nova_dir != self._ultima_dir
                and self._seconds_since(self._t_ultima_ativa) < DEAD_TIME):
            self._zerar_pwm()
            return 0.0

        mag = clamp(abs(duty), 0.0, self.duty_max)
        if nova_dir > 0:
            self._lpwm.value = 0.0
            self._rpwm.value = mag
        else:
            self._rpwm.value = 0.0
            self._lpwm.value = mag

        self._ultima_dir = nova_dir
        self._t_ultima_ativa = self.get_clock().now()
        return nova_dir * mag

    def destroy_node(self):
        self._zerar_pwm()
        self._r_en.off()
        self._l_en.off()
        self._rpwm.close()
        self._lpwm.close()
        self._r_en.close()
        self._l_en.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorPidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()