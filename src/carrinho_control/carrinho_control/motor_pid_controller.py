#!/usr/bin/env python3
"""
motor_pid_controller.py

Nó "PID" do mapa mental do carrinho:

    teleop_keyboard --pub--> /cmd_vel --sub--> [PID] --pub--> /cmd_vel_pid
                                          ^  ^                      |
                                          |  |                      +--linear.x--> [motor_node] --> motor
                                          |  |                      +--angular.z-> [servo_node] --> servo
                                          |  |
                                          |  +---- /bno055/Imu.angular_velocity.z (real do SERVO, giro filtrado)
                                          |
                                       /wheel_odom (real do MOTOR, twist.twist.linear.x)
                                          ^
                                          |
                                     [wheel_odom_relay] <-- encoder

Este nó fecha DUAS malhas PID DESACOPLADAS dentro do mesmo control_loop,
publicando o resultado combinado em um único /cmd_vel_pid:

  MALHA DO MOTOR (velocidade linear)
    setpoint = /cmd_vel.linear.x
    real     = /wheel_odom.twist.twist.linear.x
    saída    -> /cmd_vel_pid.linear.x  (motor_node só usa este campo)

  MALHA DO SERVO (direção / yaw rate)
    setpoint = /cmd_vel.angular.z
    real     = /bno055/Imu.angular_velocity.z (giro CRU, passa-baixa filtrado)
    saída    -> /cmd_vel_pid.angular.z  (servo_node só usa este campo)

CHANGELOG (revisão de segurança/robustez):
  1. Timeout de /cmd_vel: se o setpoint parar de chegar, zera as duas
     malhas e reseta os PIDs, em vez de repetir o último comando pra sempre.
  2. Malhas desacopladas de verdade: perda de wheel_odom não trava mais a
     correção do servo, e vice-versa. Cada malha tem seu próprio watchdog
     de "dado válido" (inclusive detectando dado PARADO/obsoleto, não só
     ausente).
  3. Feedback do servo trocado de "derivada do yaw fundido do BNO055" para
     "giro cru (angular_velocity.z) com filtro passa-baixa". A orientação
     fundida pode saltar por interferência magnética (motor DC, fiação,
     bateria por perto) e esse salto vira um pico de yaw-rate falso ao
     derivar — perigoso pra malha de direção. Giro cru filtrado dá resposta
     mais previsível.
  4. Clamp final absoluto na saída (setpoint + correção), além do clamp
     que já existia só na correção — evita mandar comando fisicamente
     impossível pro motor/servo quando o setpoint já está no limite.

Modelo identificado do motor DC (usado como referência pra tunar o PID do
motor; não entra diretamente no cálculo):
    K = 552.3, tau = 1.516 s, L = 0.047 s
    IMC inicial: Kp=0.002, Ki=0.001
Os ganhos do servo (servo_*) precisam de identificação própria do servo/
resposta de yaw — os valores abaixo são só placeholder até você
caracterizar essa malha também.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from carrinho_control.pid import PID, LowPassFilter


class PidController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)     # (1) segurança de cmd_vel
        self.declare_parameter('feedback_stale_sec', 0.2)  # (2) watchdog de dado parado

        # ---- PID do MOTOR (malha de velocidade linear, fechada com /wheel_odom) ----
        self.declare_parameter('motor_kp', 0.002)   # IMC do FOPDT identificado
        self.declare_parameter('motor_ki', 0.001)
        self.declare_parameter('motor_kd', 0.0)
        self.declare_parameter('motor_correction_limit', 0.5)  # m/s
        self.declare_parameter('motor_output_limit', 1.0)      # (4) clamp final, m/s

        # ---- PID do SERVO (malha de direção/yaw, fechada com /bno055) ----
        self.declare_parameter('servo_kp', 0.5)     # placeholder — identificar depois
        self.declare_parameter('servo_ki', 0.0)
        self.declare_parameter('servo_kd', 0.0)
        self.declare_parameter('servo_correction_limit', 0.4)  # rad/s
        self.declare_parameter('servo_output_limit', 1.2)      # (4) clamp final, rad/s
        self.declare_parameter('gyro_filter_alpha', 0.2)       # (3) filtro do giro cru

        motor_lim = self.get_parameter('motor_correction_limit').value
        servo_lim = self.get_parameter('servo_correction_limit').value

        self.motor_output_limit = self.get_parameter('motor_output_limit').value
        self.servo_output_limit = self.get_parameter('servo_output_limit').value

        self.cmd_timeout = self.get_parameter('cmd_timeout_sec').value
        self.feedback_stale = self.get_parameter('feedback_stale_sec').value

        self.pid_motor = PID(
            self.get_parameter('motor_kp').value,
            self.get_parameter('motor_ki').value,
            self.get_parameter('motor_kd').value,
            out_min=-motor_lim, out_max=motor_lim,
        )
        self.pid_servo = PID(
            self.get_parameter('servo_kp').value,
            self.get_parameter('servo_ki').value,
            self.get_parameter('servo_kd').value,
            out_min=-servo_lim, out_max=servo_lim,
        )

        self.gyro_filter = LowPassFilter(self.get_parameter('gyro_filter_alpha').value)

        # setpoints (de /cmd_vel)
        self.setpoint_linear_x = 0.0
        self.setpoint_angular_z = 0.0

        # valores reais medidos
        self.real_linear_x = 0.0      # de /wheel_odom            -> malha do motor
        self.real_angular_z = 0.0     # giro cru filtrado (BNO055) -> malha do servo

        # (1) watchdog do setpoint
        self.last_cmd_time = self.get_clock().now()

        # (2) watchdog independente de CADA malha de feedback
        self.last_wheel_odom_time = None
        self.last_imu_time = None

        # subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.wheel_odom_cb, 10)
        self.create_subscription(Imu, '/bno055/imu', self.imu_cb, 10)

        # publisher único — motor_node lê linear.x, servo_node lê angular.z
        self.cmd_vel_pid_pub = self.create_publisher(Twist, '/cmd_vel_pid', 10)

        rate = self.get_parameter('control_rate_hz').value
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'pid_controller iniciado | pub em /cmd_vel_pid @ {rate} Hz '
            f'(linear.x -> motor_node, angular.z -> servo_node)'
        )

    # ---------------- callbacks ----------------

    def cmd_vel_cb(self, msg: Twist):
        self.setpoint_linear_x = msg.linear.x
        self.setpoint_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()  # (1)

    def wheel_odom_cb(self, msg: Odometry):
        self.real_linear_x = msg.twist.twist.linear.x
        self.last_wheel_odom_time = self.get_clock().now()  # (2)

    def imu_cb(self, msg: Imu):
        # (3) giro cru (rad/s), passado por filtro passa-baixa pra tirar ruído
        # de alta frequência sem sofrer salto de correção magnética como a
        # orientação fundida podia causar.
        self.real_angular_z = self.gyro_filter.update(msg.angular_velocity.z)
        self.last_imu_time = self.get_clock().now()  # (2)

    # ---------------- helpers de watchdog ----------------

    def _seconds_since(self, ros_time):
        if ros_time is None:
            return float('inf')
        return (self.get_clock().now() - ros_time).nanoseconds / 1e9

    # ---------------- laço de controle (2 PIDs desacoplados) ----------------

    def control_loop(self):
        # (1) segurança: sem /cmd_vel novo há muito tempo -> zera tudo
        if self._seconds_since(self.last_cmd_time) > self.cmd_timeout:
            self.setpoint_linear_x = 0.0
            self.setpoint_angular_z = 0.0
            self.pid_motor.reset()
            self.pid_servo.reset()

        motor_ok = self._seconds_since(self.last_wheel_odom_time) <= self.feedback_stale
        servo_ok = self._seconds_since(self.last_imu_time) <= self.feedback_stale

        out_msg = Twist()

        # ---- malha do motor (independente da malha do servo) ----
        if motor_ok:
            error_linear = self.setpoint_linear_x - self.real_linear_x
            correction_linear = self.pid_motor.update(error_linear, self.dt)
            linear_cmd = self.setpoint_linear_x + correction_linear
        else:
            # sem feedback confiável: não inventa correção, e não acumula erro
            self.pid_motor.reset()
            linear_cmd = self.setpoint_linear_x

        # ---- malha do servo (independente da malha do motor) ----
        if servo_ok:
            error_angular = self.setpoint_angular_z - self.real_angular_z
            correction_angular = self.pid_servo.update(error_angular, self.dt)
            angular_cmd = self.setpoint_angular_z + correction_angular
        else:
            self.pid_servo.reset()
            angular_cmd = self.setpoint_angular_z

        # (4) clamp final absoluto, não só da correção
        out_msg.linear.x = max(-self.motor_output_limit,
                                min(self.motor_output_limit, linear_cmd))
        out_msg.angular.z = max(-self.servo_output_limit,
                                 min(self.servo_output_limit, -angular_cmd))

        self.cmd_vel_pid_pub.publish(out_msg)

        if not motor_ok:
            self.get_logger().warn('wheel_odom obsoleto/ausente — motor sem correção PID',
                                    throttle_duration_sec=2.0)
        if not servo_ok:
            self.get_logger().warn('IMU obsoleta/ausente — servo sem correção PID',
                                    throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PidController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
