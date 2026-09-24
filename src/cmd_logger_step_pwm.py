#!/usr/bin/env python3
"""
logger_step_csv.py

Roda na Raspberry Pi. Grava, num ÚNICO arquivo CSV de formato LARGO
(colunas separadas, não mais o formato "melted" tempo/grandeza/valor):

    /rpm           (std_msgs/Int32)           -> coluna 'rpm'
    /wheel_odom    (nav_msgs/Odometry)         -> colunas 'wheel_odom_linear_x',
                                                   'wheel_odom_stamp_msg_s'
    /cmd_vel_pid   (geometry_msgs/Twist)       -> coluna 'cmd_vel_pid_linear_x'
    /cmd_vel       (geometry_msgs/Twist)       -> coluna 'cmd_vel_linear_x' (ADICIONADO:
                                                   setpoint bruto, usado no teste de
                                                   resposta ao degrau publicado direto
                                                   via `ros2 topic pub /cmd_vel ...`)

Cada linha do CSV é escrita toda vez que QUALQUER um dos três tópicos
publica uma mensagem nova — e carrega o ÚLTIMO valor conhecido de TODOS
os campos (não só do tópico que acabou de chegar). Isso monta uma tabela
"wide" sincronizada sem precisar de interpolação: basta pegar os valores
mais recentes de cada coluna a cada evento.

O eixo de tempo usado é sempre time.time() do PRÓPRIO Raspberry Pi no
momento em que a linha é escrita (não o header.stamp do ESP32) — porque
/cmd_vel_pid (geometry_msgs/Twist) nem tem campo de timestamp, então
usar o relógio do Pi pra tudo mantém os três sinais na MESMA base de
tempo, o que é o que importa pra identificação de degrau.

Uso:
    python3 logger_step_csv.py

Gera um arquivo tipo: step_1758030000.csv
"""

import csv
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class LoggerStepCsv(Node):
    def __init__(self):
        super().__init__('logger_step_csv')

        self.t0 = time.time()
        nome_arquivo = f'/home/epiibots/step_{int(self.t0)}.csv'

        self.arquivo = open(nome_arquivo, 'w', newline='')
        self.writer = csv.writer(self.arquivo)
        self.writer.writerow([
            'tempo_relativo_s',        # eixo de tempo principal (relógio do Pi)
            'rpm',                     # último valor conhecido de /rpm
            'wheel_odom_linear_x',     # último valor conhecido de /wheel_odom
            'wheel_odom_stamp_msg_s',  # timestamp DA MENSAGEM do wheel_odom (ESP32)
            'cmd_vel_pid_linear_x',    # último valor conhecido de /cmd_vel_pid
            'cmd_vel_linear_x',        # ADICIONADO: último valor conhecido de /cmd_vel (setpoint bruto, usado no teste de degrau)
            'debug_motor_pwm',         # ADICIONADO: duty cycle PWM real (/debug/motor_pwm, do motor_node)
        ])
        self.arquivo.flush()

        self.nome_arquivo = nome_arquivo

        # últimos valores conhecidos de cada sinal (começam em 0/None)
        self.ultimo_rpm = 0
        self.ultimo_wheel_odom_x = 0.0
        self.ultimo_wheel_odom_stamp_msg = 0.0
        self.ultimo_cmd_pid_x = 0.0
        self.ultimo_cmd_vel_x = 0.0  # ADICIONADO
        self.ultimo_pwm = 0.0        # ADICIONADO

        self.create_subscription(Int32, '/rpm', self.cb_rpm, 10)
        self.create_subscription(Odometry, '/wheel_odom', self.cb_wheel_odom, 10)
        self.create_subscription(Twist, '/cmd_vel_pid', self.cb_cmd_pid, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)  # ADICIONADO
        self.create_subscription(Float32, '/debug/motor_pwm', self.cb_pwm, 10)  # ADICIONADO

        self.get_logger().info(f'Gravando em: {nome_arquivo}')

    def _escrever_linha(self):
        t = time.time() - self.t0
        self.writer.writerow([
            f'{t:.6f}',
            self.ultimo_rpm,
            self.ultimo_wheel_odom_x,
            f'{self.ultimo_wheel_odom_stamp_msg:.6f}',
            self.ultimo_cmd_pid_x,
            self.ultimo_cmd_vel_x,  # ADICIONADO
            self.ultimo_pwm,        # ADICIONADO
        ])
        self.arquivo.flush()

    def cb_rpm(self, msg):
        self.ultimo_rpm = msg.data
        self._escrever_linha()

    def cb_wheel_odom(self, msg: Odometry):
        self.ultimo_wheel_odom_x = msg.twist.twist.linear.x
        self.ultimo_wheel_odom_stamp_msg = (
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        )
        self._escrever_linha()

    def cb_cmd_pid(self, msg: Twist):
        self.ultimo_cmd_pid_x = msg.linear.x
        self._escrever_linha()

    def cb_cmd_vel(self, msg: Twist):  # ADICIONADO
        self.ultimo_cmd_vel_x = msg.linear.x
        self._escrever_linha()

    def cb_pwm(self, msg: Float32):  # ADICIONADO
        self.ultimo_pwm = msg.data
        self._escrever_linha()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerStepCsv()
    print(f'Gravando... Ctrl+C para parar. Arquivo: {node.nome_arquivo}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.arquivo.close()
    print(f'Arquivo salvo: {node.nome_arquivo}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()