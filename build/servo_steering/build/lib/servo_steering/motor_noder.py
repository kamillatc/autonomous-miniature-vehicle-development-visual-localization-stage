#!/usr/bin/env python3
"""
motor_node.py — com publicação de PWM para análise velocidade×PWM
Tópicos publicados:
  /motor/pwm_duty   (std_msgs/Int32) → PWM em escala 0-255 (estilo Arduino)
                                        negativo = ré, positivo = frente, 0 = parado
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# ── Configuração dos pinos ────────────────────────────────────────────────────
PIN_RPWM = 6
PIN_LPWM = 13
PIN_REN  = 2
PIN_LEN  = 3

# ── Parâmetros do motor ───────────────────────────────────────────────────────
MAX_LINEAR_MS = 1.0
PWM_FREQ      = 1000
DEAD_TIME     = 0.05

V_BATERIA   = 12.0
V_MAX_MOTOR = 6.5
DUTY_MAX    = min(V_MAX_MOTOR / V_BATERIA, 1.0)
# ─────────────────────────────────────────────────────────────────────────────


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        self._rpwm = PWMOutputDevice(PIN_RPWM, frequency=PWM_FREQ)
        self._lpwm = PWMOutputDevice(PIN_LPWM, frequency=PWM_FREQ)
        self._r_en = DigitalOutputDevice(PIN_REN)
        self._l_en = DigitalOutputDevice(PIN_LEN)

        self._r_en.on()
        self._l_en.on()
        self._direcao_atual = 0
        self._duty_atual = 0.0  # armazena o duty com sinal para publicar

        self._zerar_pwm()

        # Subscriber
        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)

        # Publisher de PWM em escala 0-255 — sinalizado: positivo=frente, negativo=ré
        self._pub_pwm = self.create_publisher(Int32, "/motor/pwm_duty", 10)

        self.get_logger().info("motor_node pronto — aguardando /cmd_vel")

    # ── Callback ──────────────────────────────────────────────────────────────
    def _cb(self, msg: Twist):
        velocidade_pct = (msg.linear.x / MAX_LINEAR_MS) * 100.0
        velocidade_pct = max(-100.0, min(100.0, velocidade_pct))
        self._acionar(velocidade_pct)

    # ── Controle do motor ─────────────────────────────────────────────────────
    def _zerar_pwm(self):
        self._rpwm.value = 0
        self._lpwm.value = 0

    def _publicar_pwm(self, duty_com_sinal: float):
        """
        Publica PWM em escala 0-255 (estilo Arduino) com sinal em /motor/pwm_duty.
        duty_com_sinal: fração -1.0 a +1.0 → convertida para -255 a +255.
        """
        self._duty_atual = duty_com_sinal
        pwm_255 = int(round(duty_com_sinal * 255))
        msg = Int32()
        msg.data = pwm_255
        self._pub_pwm.publish(msg)

    def _acionar(self, velocidade: float):
        potencia = (abs(velocidade) / 100.0) * DUTY_MAX
        nova_direcao = 1 if velocidade > 0 else (-1 if velocidade < 0 else 0)

        if (nova_direcao != 0
                and self._direcao_atual != 0
                and nova_direcao != self._direcao_atual):
            self._zerar_pwm()
            self._publicar_pwm(0.0)
            sleep(DEAD_TIME)

        self._direcao_atual = nova_direcao

        if velocidade > 0:
            self._lpwm.value = 0
            self._rpwm.value = potencia
            self._publicar_pwm(+potencia)   # ← duty positivo = frente
            self.get_logger().info(
                f"FRENTE {velocidade:.1f}% → PWM {int(round(potencia*255))}/255 "
                f"(duty {potencia*100:.1f}%)")

        elif velocidade < 0:
            self._rpwm.value = 0
            self._lpwm.value = potencia
            self._publicar_pwm(-potencia)   # ← duty negativo = ré
            self.get_logger().info(
                f"TRÁS   {abs(velocidade):.1f}% → PWM {int(round(potencia*255))}/255 "
                f"(duty {potencia*100:.1f}%)")

        else:
            self._parar()

    def _parar(self):
        self._zerar_pwm()
        self._publicar_pwm(0.0)
        self._direcao_atual = 0
        self.get_logger().info("PARADO")

    # ── Destruição segura ─────────────────────────────────────────────────────
    def destroy_node(self):
        self._parar()
        sleep(0.1)
        self._r_en.off()
        self._l_en.off()
        self._rpwm.close()
        self._lpwm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
