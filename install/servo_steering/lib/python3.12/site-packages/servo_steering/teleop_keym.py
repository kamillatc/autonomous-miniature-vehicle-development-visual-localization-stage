#!/usr/bin/env python3
"""
teleop_key.py — ROS2 Jazzy
Publica em /cmd_vel para controlar motor_node e servo_node.

  w / s  →  linear.x   (+/- STEP_LINEAR)
  a / d  →  angular.z  (+/- STEP_ANGULAR)
  espaço →  parada total
  q      →  sair
"""

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

TOPIC        = '/cmd_vel'
STEP_LINEAR  = 0.1
STEP_ANGULAR = 0.1
MAX_LINEAR   = 1.0
MAX_ANGULAR  = 1.0

MSG = """
┌──────────────────────────────────────┐
│         Teleop Key — /cmd_vel        │
│                                      │
│   w = frente       s = ré            │
│   a = esquerda     d = direita       │
│   espaço = parar   q = sair          │
└──────────────────────────────────────┘
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self._pub     = self.create_publisher(Twist, TOPIC, 10)
        self._linear  = 0.0
        self._angular = 0.0
        print(MSG)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = get_key(settings)

                if key == 'w':
                    self._linear = min(self._linear + STEP_LINEAR, MAX_LINEAR)
                elif key == 's':
                    self._linear = max(self._linear - STEP_LINEAR, -MAX_LINEAR)
                elif key == 'a':
                    self._angular = min(self._angular + STEP_ANGULAR, MAX_ANGULAR)
                elif key == 'd':
                    self._angular = max(self._angular - STEP_ANGULAR, -MAX_ANGULAR)
                elif key == ' ':
                    self._linear  = 0.0
                    self._angular = 0.0
                elif key in ('q', '\x03'):
                    break
                else:
                    return

                msg = Twist()
                msg.linear.x  = self._linear
                msg.angular.z = self._angular
                self._pub.publish(msg)

                print(
                    f'\r  linear.x: {self._linear:+.1f}  '
                    f'angular.z: {self._angular:+.1f}    ',
                    end='', flush=True
                )
        finally:
            # Garante parada ao sair
            stop = Twist()
            self._pub.publish(stop)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print('\nParado.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()