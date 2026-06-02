#!/usr/bin/env python3
"""
teleop_key.py — ROS2 Jazzy
w/s = frente/ré (zera ao soltar)
a/d = esquerda/direita (zera ao soltar)
Suporta w+a, w+d, s+a, s+d simultaneamente
espaço = parar | q = sair
"""

import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

TOPIC      = '/cmd_vel'
PUBLISH_HZ = 10

MSG = """
┌──────────────────────────────────────┐
│         Teleop Key — /cmd_vel        │
│                                      │
│   w = frente  (-0.9)                 │
│   s = ré      (+0.9)                 │
│   a = esquerda (-0.8)                │
│   d = direita  (+0.8)                │
│                                      │
│   Combinações: w+a, w+d, s+a, s+d   │
│   Todas as teclas zeram ao soltar    │
│   espaço = parar | q = sair          │
└──────────────────────────────────────┘
"""


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')
        self._pub  = self.create_publisher(Twist, TOPIC, 10)
        self._lock = threading.Lock()

        # Teclas atualmente pressionadas
        self._pressed = set()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        self._listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self._listener.start()

        print(MSG)

    def _on_press(self, key):
        try:
            k = key.char
        except AttributeError:
            if key == keyboard.Key.space:
                with self._lock:
                    self._pressed.clear()
                self._print_state(0.0, 0.0)
            return

        if k in ('w', 's', 'a', 'd'):
            with self._lock:
                # Remove oposto para não ter w+s ou a+d ao mesmo tempo
                if k == 'w': self._pressed.discard('s')
                if k == 's': self._pressed.discard('w')
                if k == 'a': self._pressed.discard('d')
                if k == 'd': self._pressed.discard('a')
                self._pressed.add(k)
        elif k == 'q':
            with self._lock:
                self._pressed.clear()
            self._publish()
            rclpy.shutdown()

    def _on_release(self, key):
        try:
            k = key.char
        except AttributeError:
            return

        if k in ('w', 's', 'a', 'd'):
            with self._lock:
                self._pressed.discard(k)

    def _compute(self):
        linear  = 0.0
        angular = 0.0
        with self._lock:
            pressed = set(self._pressed)

        if 'w' in pressed: linear  = -0.9
        if 's' in pressed: linear  = +1.0
        if 'a' in pressed: angular = -0.8
        if 'd' in pressed: angular = +0.8

        return linear, angular

    def _publish(self):
        linear, angular = self._compute()
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._pub.publish(msg)
        self._print_state(linear, angular)

    def _print_state(self, linear, angular):
        print(
            f'\r  linear.x: {linear:+.1f}  angular.z: {angular:+.1f}    ',
            end='', flush=True
        )


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node._pub.publish(stop)
        node._listener.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()