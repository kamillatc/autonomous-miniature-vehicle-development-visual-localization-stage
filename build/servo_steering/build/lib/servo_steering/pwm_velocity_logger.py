#!/usr/bin/env python3
"""
pwm_velocity_logger.py
Nó ROS2 simples: assina /motor/pwm_duty (Int32, -255 a 255) e /RPM (Float32),
e grava cada amostra numa lista que é salva em CSV ao final.
Sem matplotlib, sem servidor web — só coleta os dados para você plotar depois
(Excel, Google Sheets, outro script, etc).

Uso:
  python3 pwm_velocity_logger.py [--rpm_topic /RPM] [--output dados.csv]

Pressione Ctrl+C para parar e salvar o CSV.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32

import csv
import math
import time
import argparse
import signal
import sys
import os

# ── Configurações do encoder ──────────────────────────────────────────────────
GEAR_RATIO   = 1.0      # Relação de transmissão (ajuste se houver redutor)
WHEEL_DIAM_M = 0.065    # Diâmetro da roda em metros (ajuste para o seu carro)
WHEEL_CIRCUM = math.pi * WHEEL_DIAM_M
# ─────────────────────────────────────────────────────────────────────────────


def rpm_to_ms(rpm: float) -> float:
    """Converte RPM do encoder para m/s na roda."""
    return (rpm / 60.0) * WHEEL_CIRCUM / GEAR_RATIO


class LoggerNode(Node):
    def __init__(self, rpm_topic: str):
        super().__init__("pwm_velocity_logger_node")

        self.t0 = time.time()
        self.last_pwm = 0
        self.last_vel = 0.0

        # Lista que guarda cada amostra: (tempo_s, pwm_0_255, velocidade_ms)
        self.dados = []

        self.create_subscription(Int32, "/motor/pwm_duty", self._cb_pwm, 10)
        self.create_subscription(Float32, "/encoder_vel", self._cb_rpm, 10)

        self.get_logger().info(
            f"Logger pronto. PWM: /motor/pwm_duty | Velocidade: {rpm_topic}"
        )
        self.get_logger().info("Coletando amostras... Ctrl+C para salvar e sair.")

    def _registrar(self):
        t = time.time() - self.t0
        self.dados.append((round(t, 4), self.last_pwm, round(self.last_vel, 4)))

        # Feedback no terminal a cada 20 amostras, pra não floodar
        if len(self.dados) % 20 == 0:
            self.get_logger().info(
                f"{len(self.dados)} amostras | último: PWM={self.last_pwm:+d} "
                f"vel={self.last_vel:+.3f} m/s"
            )

    def _cb_pwm(self, msg: Int32):
        self.last_pwm = int(msg.data)
        self._registrar()

    def _cb_rpm(self, msg: Float32):
        self.last_vel = rpm_to_ms(float(msg.data))
        self._registrar()

    def salvar_csv(self, path: str):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["tempo_s", "pwm_0_255", "velocidade_ms"])
            w.writerows(self.dados)
        print(f"\n{len(self.dados)} amostras salvas em: {os.path.abspath(path)}")


def main():
    parser = argparse.ArgumentParser(description="Logger PWM x Velocidade para CSV")
    parser.add_argument("--rpm_topic", default="/RPM", help="Tópico RPM do encoder")
    parser.add_argument("--output", default="dados.csv", help="Arquivo CSV de saída")
    args = parser.parse_args()

    rclpy.init()
    node = LoggerNode(rpm_topic=args.rpm_topic)

    def shutdown(*_):
        node.salvar_csv(args.output)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    main()
