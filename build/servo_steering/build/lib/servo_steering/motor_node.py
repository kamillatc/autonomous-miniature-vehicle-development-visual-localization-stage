#!/usr/bin/env python3
"""
motor_node.py
Assina /cmd_vel (geometry_msgs/Twist) — mesmo tópico que o Nav2 publica.
  msg.linear.x  →  velocidade linear em m/s
                   convertida para duty cycle PWM (-100% a 100%)
Conversão:  velocidade_pct = (linear.x / MAX_LINEAR_MS) * 100
  Ex: linear.x = 0.5 m/s com MAX=1.0  →  50%
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep

# ── Configuração dos pinos ────────────────────────────────────────────────────
PIN_RPWM = 6    # Sentido Frente
PIN_LPWM = 13   # Sentido Trás
PIN_REN  = 2    # Enable Direito (R_EN)
PIN_LEN  = 3    # Enable Esquerdo (L_EN)

# ── Parâmetros do motor ───────────────────────────────────────────────────────
MAX_LINEAR_MS = 1.0   # m/s que corresponde a 100% de duty cycle
PWM_FREQ      = 1000  # Hz — ideal para BTS7960 com motores DC
DEAD_TIME     = 0.05  # 50ms de proteção na inversão de sentido

V_BATERIA      = 12.0  # Tensão real da sua bateria em volts  ← ajuste aqui
V_MAX_MOTOR    = 6  # Tensão máxima desejada no motor em volts

# Duty cycle máximo permitido (ex: 7.5/12.0 = 0.625 → 62.5%)
DUTY_MAX = min(V_MAX_MOTOR / V_BATERIA, 1.0)
# ─────────────────────────────────────────────────────────────────────────────

class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        # Inicializa hardware
        self._rpwm  = PWMOutputDevice(PIN_RPWM, frequency=PWM_FREQ)
        self._lpwm  = PWMOutputDevice(PIN_LPWM, frequency=PWM_FREQ)
        self._r_en  = DigitalOutputDevice(PIN_REN)
        self._l_en  = DigitalOutputDevice(PIN_LEN)

        self._r_en.on()
        self._l_en.on()

        self._direcao_atual = 0  # -1 = trás, 0 = parado, 1 = frente

        self._zerar_pwm()

        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.get_logger().info("motor_node pronto — aguardando /cmd_vel")

    # ── Callback ──────────────────────────────────────────────────────────────
    def _cb(self, msg: Twist):
        # linear.x em m/s → porcentagem de -100 a 100
        velocidade_pct = (msg.linear.x / MAX_LINEAR_MS) * 100.0
        velocidade_pct = max(-100.0, min(100.0, velocidade_pct))
        self._acionar(velocidade_pct)

    # ── Controle do motor ─────────────────────────────────────────────────────
    def _zerar_pwm(self):
        """Zera AMBOS os canais. Nunca pule antes de uma transição."""
        self._rpwm.value = 0
        self._lpwm.value = 0

    def _acionar(self, velocidade: float):
        """
        velocidade: -100.0 a 100.0
          Positivo → Frente (RPWM ativo, LPWM = 0)
          Negativo → Trás   (LPWM ativo, RPWM = 0)
          Zero     → Para

        REGRA CRÍTICA BTS7960: nunca ative RPWM e LPWM simultaneamente.
        """
        potencia = (abs(velocidade) / 100.0) * DUTY_MAX
        nova_direcao = 1 if velocidade > 0 else (-1 if velocidade < 0 else 0)

        # Dead-time na inversão de sentido
        if (nova_direcao != 0
                and self._direcao_atual != 0
                and nova_direcao != self._direcao_atual):
            self._zerar_pwm()
            sleep(DEAD_TIME)

        self._direcao_atual = nova_direcao

        if velocidade > 0:
            self._lpwm.value = 0
            self._rpwm.value = potencia
            self.get_logger().info(f"linear.x → FRENTE {velocidade:.1f}%")

        elif velocidade < 0:
            self._rpwm.value = 0
            self._lpwm.value = potencia
            self.get_logger().info(f"linear.x → TRÁS   {abs(velocidade):.1f}%")

        else:
            self._parar()

    def _parar(self):
        self._zerar_pwm()
        self._direcao_atual = 0
        self.get_logger().info("linear.x → PARADO")

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