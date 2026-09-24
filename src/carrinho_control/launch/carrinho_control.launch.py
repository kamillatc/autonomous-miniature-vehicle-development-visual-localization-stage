#!/usr/bin/env python3
"""
carrinho_control.launch.py

Sobe a pilha de controle completa, exceto o teleop (que é interativo e
deve ser rodado à parte, manualmente, num terminal em foco):

    wheel_odom_relay (carrinho_localization)
    motor_node       (servo_steering)
    servo_node       (servo_steering)
    motor_pid_controller (carrinho_control)

IMPORTANTE: os nomes 'motor_node', 'servo_node' e 'wheel_odom_relay' abaixo
assumem que os entry_points nos outros dois pacotes têm o mesmo nome do
arquivo .py. Se você deu nome diferente no setup.py deles, ajuste aqui.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='carrinho_localization',
            executable='wheel_odom_relay',
            name='wheel_odom_relay',
            output='screen',
        ),
        Node(
            package='servo_steering',
            executable='motor_node',
            name='motor_node',
            output='screen',
        ),
        Node(
            package='servo_steering',
            executable='servo_node',
            name='servo_node',
            output='screen',
        ),
        Node(
            package='carrinho_control',
            executable='motor_pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[
                # dá pra apontar um yaml aqui no futuro em vez de hardcode,
                # ex: os.path.join(get_package_share_directory('carrinho_control'), 'config', 'pid.yaml')
            ],
        ),
    ])
