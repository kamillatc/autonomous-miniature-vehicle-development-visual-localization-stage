
































































































































































































































#!/usr/bin/env python3
"""
pwm_velocity_plot.py
Nó ROS2 que assina /motor/pwm_duty e /RPM (ou /velocity),
plota em tempo real o gráfico Velocidade × PWM e salva CSV ao encerrar.

Tópicos esperados:
  /motor/pwm_duty   (Float32) — duty cycle com sinal: -DUTY_MAX a +DUTY_MAX
  /RPM              (Float32) — RPM do encoder (seu nó existente)

Uso:
  python3 pwm_velocity_plot.py [--rpm_topic /RPM] [--output dados.csv]

Dependências (Pi 5):
  pip install matplotlib --break-system-packages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import csv
import argparse
import threading
import time
import os

# ── Configurações do encoder ──────────────────────────────────────────────────
PPR          = 1000     # Pulsos por revolução do E6B2-CWZ3E
GEAR_RATIO   = 1.0      # Relação de transmissão (ajuste se houver redutor)
WHEEL_DIAM_M = 0.065    # Diâmetro da roda em metros (ajuste para o seu carro)
WHEEL_CIRCUM = np.pi * WHEEL_DIAM_M  # Circunferência
# ─────────────────────────────────────────────────────────────────────────────

WINDOW_SIZE = 200       # Número de amostras visíveis no gráfico
CSV_MAX_ROWS = 10_000   # Limite de linhas no CSV


def rpm_to_ms(rpm: float) -> float:
    """Converte RPM do encoder para m/s na roda."""
    return (rpm / 60.0) * WHEEL_CIRCUM / GEAR_RATIO


class PlotNode(Node):
    def __init__(self, rpm_topic: str):
        super().__init__("pwm_velocity_plot_node")

        # Buffers de dados
        self.times   = []
        self.pwm_buf = []
        self.vel_buf = []
        self._lock   = threading.Lock()
        self._t0     = time.time()

        self._last_pwm = 0.0
        self._last_vel = 0.0

        # Subscribers
        self.create_subscription(Float32, "/motor/pwm_duty", self._cb_pwm, 10)
        self.create_subscription(Float32, rpm_topic,         self._cb_rpm, 10)

        self.get_logger().info(
            f"PlotNode pronto. PWM: /motor/pwm_duty | Velocidade: {rpm_topic}")

    def _cb_pwm(self, msg: Float32):
        self._last_pwm = float(msg.data)
        self._record()

    def _cb_rpm(self, msg: Float32):
        self._last_vel = rpm_to_ms(float(msg.data))
        self._record()

    def _record(self):
        t = time.time() - self._t0
        with self._lock:
            self.times.append(t)
            self.pwm_buf.append(self._last_pwm * 100.0)   # converte para %
            self.vel_buf.append(self._last_vel)
            # Mantém janela deslizante
            if len(self.times) > CSV_MAX_ROWS:
                self.times.pop(0)
                self.pwm_buf.pop(0)
                self.vel_buf.pop(0)


def save_csv(node: PlotNode, path: str):
    """Salva os dados coletados em CSV."""
    with node._lock:
        rows = list(zip(node.times, node.pwm_buf, node.vel_buf))
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["tempo_s", "pwm_pct", "velocidade_ms"])
        w.writerows(rows)
    print(f"\nCSV salvo em: {os.path.abspath(path)}")


def run_plot(node: PlotNode, output_csv: str):
    """Configura e exibe o gráfico animado."""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle("Velocidade × PWM — Motor DC (BTS7960)", fontsize=13, fontweight="bold")

    # ── Ax1: PWM ao longo do tempo ────────────────────────────────────────────
    line_pwm, = ax1.plot([], [], color="#1f77b4", linewidth=1.5, label="PWM (%)")
    ax1.set_ylabel("Duty Cycle (%)")
    ax1.set_ylim(-75, 75)
    ax1.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax1.legend(loc="upper right", fontsize=9)
    ax1.grid(True, alpha=0.3)

    # ── Ax2: Velocidade ao longo do tempo ────────────────────────────────────
    line_vel, = ax2.plot([], [], color="#d62728", linewidth=1.5, label="Velocidade (m/s)")
    ax2.set_ylabel("Velocidade (m/s)")
    ax2.set_ylim(-2.5, 2.5)
    ax2.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax2.legend(loc="upper right", fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel("Tempo (s)")

    # ── Ax3: Scatter PWM × Velocidade ─────────────────────────────────────────
    sc = ax3.scatter([], [], s=4, alpha=0.5, color="#2ca02c")
    ax3.set_xlabel("Duty Cycle (%)")
    ax3.set_ylabel("Velocidade (m/s)")
    ax3.set_title("Curva característica PWM × Velocidade", fontsize=10)
    ax3.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax3.axvline(0, color="gray", linewidth=0.5, linestyle="--")
    ax3.set_xlim(-75, 75)
    ax3.set_ylim(-2.5, 2.5)
    ax3.grid(True, alpha=0.3)

    # Linha de regressão linear
    line_reg, = ax3.plot([], [], "r--", linewidth=1.2, label="Regressão linear")
    ax3.legend(loc="upper left", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

    def update(_frame):
        with node._lock:
            if len(node.times) < 2:
                return line_pwm, line_vel, sc, line_reg

            ts   = np.array(node.times[-WINDOW_SIZE:])
            pwms = np.array(node.pwm_buf[-WINDOW_SIZE:])
            vels = np.array(node.vel_buf[-WINDOW_SIZE:])

        # Gráficos temporais
        line_pwm.set_data(ts, pwms)
        ax1.set_xlim(ts[0], max(ts[-1], ts[0] + 1))

        line_vel.set_data(ts, vels)
        ax2.set_xlim(ts[0], max(ts[-1], ts[0] + 1))

        # Scatter: todos os dados
        with node._lock:
            all_pwm = np.array(node.pwm_buf)
            all_vel = np.array(node.vel_buf)

        sc.set_offsets(np.c_[all_pwm, all_vel])

        # Regressão linear (apenas se houver dados suficientes e variação)
        if len(all_pwm) > 20 and np.std(all_pwm) > 0.5:
            coeffs = np.polyfit(all_pwm, all_vel, 1)
            x_fit = np.linspace(-65, 65, 100)
            y_fit = np.polyval(coeffs, x_fit)
            line_reg.set_data(x_fit, y_fit)
            ax3.set_title(
                f"Curva PWM × Velocidade  |  k={coeffs[0]:.4f} m/s/% "
                f"  offset={coeffs[1]:.3f} m/s",
                fontsize=10,
            )

        return line_pwm, line_vel, sc, line_reg

    ani = animation.FuncAnimation(fig, update, interval=100, blit=False)

    try:
        plt.show()
    finally:
        save_csv(node, output_csv)


def main():
    parser = argparse.ArgumentParser(description="Plot Velocidade×PWM em tempo real")
    parser.add_argument("--rpm_topic", default="/RPM",   help="Tópico RPM do encoder")
    parser.add_argument("--output",    default="pwm_vel.csv", help="Arquivo CSV de saída")
    args = parser.parse_args()

    rclpy.init()
    node = PlotNode(rpm_topic=args.rpm_topic)

    # Spin em thread separada (matplotlib precisa da thread principal)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_plot(node, args.output)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
