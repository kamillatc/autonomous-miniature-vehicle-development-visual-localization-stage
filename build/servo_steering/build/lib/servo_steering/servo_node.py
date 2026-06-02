#!/usr/bin/env python3
"""
servo_node.py
Assina /cmd_vel (geometry_msgs/Twist) — mesmo tópico que o Nav2 publica.

  msg.angular.z  →  velocidade angular em rad/s
                    convertida para ângulo de esterço em graus

Conversão:  angle_deg = angular.z * (180 / pi)
  Ex: angular.z = 0.52 rad/s  →  ~30°

Cole os valores do servo_calibration.txt nas constantes abaixo.
"""

import math
import lgpio
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ── Cole aqui os valores do servo_calibration.txt ────────────────────────────
SERVO_MIN_US    = 2000
SERVO_MAX_US    = 4010
SERVO_CENTER_US = 3000
ANGLE_MAX_DEG   = 29
ANGLE_MIN_DEG   = -29
# ───────────────────────────────────────────────────────────────────────────

GPIO_PIN = 18
PWM_FREQ = 100


def angle_to_us(angle_deg: float) -> float:
    angle_deg = max(ANGLE_MIN_DEG, min(ANGLE_MAX_DEG, angle_deg))
    if angle_deg < 0:
        us = SERVO_CENTER_US + (angle_deg / ANGLE_MIN_DEG) * (SERVO_MIN_US - SERVO_CENTER_US)
    else:
        us = SERVO_CENTER_US + (angle_deg / ANGLE_MAX_DEG) * (SERVO_MAX_US - SERVO_CENTER_US)
    return us


class ServoNode(Node):
    def __init__(self):
        super().__init__("servo_node")

        self._h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(self._h, GPIO_PIN)
        self._move(0.0)


        self.create_subscription(Twist, "/cmd_vel", self._cb, 10)
        self.get_logger().info("servo_node pronto — aguardando /cmd_vel")

    def _cb(self, msg: Twist):
        # angular.z em rad/s → graus
        angle_deg = math.degrees(msg.angular.z)
        self._move(angle_deg)

    def _move(self, angle_deg: float):
        us = angle_to_us(angle_deg)
        duty = (us / 20000.0) * 100
        lgpio.tx_pwm(self._h, GPIO_PIN, PWM_FREQ, duty)
        self.get_logger().info(f"angular.z → {angle_deg:.1f}° → {us:.0f} µs")

    def destroy_node(self):
        self._move(0.0)
        import time; time.sleep(0.3)
        lgpio.tx_pwm(self._h, GPIO_PIN, 0, 0)
        lgpio.gpiochip_close(self._h)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
