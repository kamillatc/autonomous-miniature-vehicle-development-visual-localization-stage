#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry   # ALTERADO: era TwistWithCovarianceStamped



class WheelOdomRelay(Node):
    def __init__(self):
        super().__init__('wheel_odom_relay')

        # Assina o dado cru e leve vindo do ESP32
        self.sub = self.create_subscription(
            TwistStamped,
            'wheel_odom_raw',
            self.callback,
            10
        )

        # Publica já estruturado com covariância, pronto pro robot_localization
        self.pub = self.create_publisher(
            Odometry,          # ALTERADO: agora publica Odometry completo
            'wheel_odom',
            10
        )

        self.get_logger().info('wheel_odom_relay iniciado: /wheel_odom_raw -> /wheel_odom')

    def callback(self, msg: TwistStamped):
        out = Odometry()   # ALTERADO

        # IMPORTANTE: reaproveita o timestamp ORIGINAL do ESP32,
        # não gera um novo -- preserva o instante real da medição
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'          # ADICIONADO: frame de referência da odometria
        out.child_frame_id = 'base_link'      # ADICIONADO: nav_msgs/Odometry exige esse campo

        # Copia a velocidade medida
        out.twist.twist.linear.x = msg.twist.linear.x
        out.twist.twist.linear.y = 0.0
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = 0.0

        # Covariância: baixa em linear.x (confiável), alta no resto (não medido)
        cov = [0.0] * 36
        cov[0] = 0.01      # linear.x
        cov[7] = 99999.0   # linear.y
        cov[14] = 99999.0  # linear.z
        cov[21] = 99999.0  # angular.x
        cov[28] = 99999.0  # angular.y
        cov[35] = 99999.0  # angular.z
        out.twist.covariance = cov


        # ADICIONADO: pose não é medida por esse sensor, então deixamos com
        # covariância altíssima em tudo, pra dizer ao EKF "ignore esses campos"
        out.pose.covariance = [99999.0] * 36

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()