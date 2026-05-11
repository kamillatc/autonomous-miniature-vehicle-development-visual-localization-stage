import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
import yaml

class MultiSensorSaver(Node):
    def __init__(self):
        super().__init__('multi_sensor_saver')
        
        self.data_log = []
        self.saving = False

        # Subscribers usando message_filters
        self.lidar_sub = Subscriber(self, LaserScan, '/scan')
        self.imu_sub = Subscriber(self, Imu, '/bno055/imu')

        # Sincronizador aproximado
        self.ts = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.imu_sub],
            queue_size=20,
            slop=0.05  # tolerância de 50ms (ajuste conforme necessário)
        )

        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, lidar_msg, imu_msg):
        if not self.saving:
            return

        # Timestamp do ROS (use o do lidar como referência)
        timestamp = (
            lidar_msg.header.stamp.sec +
            lidar_msg.header.stamp.nanosec * 1e-9
        )

        self.data_log.append({
            'timestamp': timestamp,
            'lidar': {
                'angle_min': lidar_msg.angle_min,
                'angle_max': lidar_msg.angle_max,
                'angle_increment': lidar_msg.angle_increment,
                'ranges': list(lidar_msg.ranges),
                'intensities': list(lidar_msg.intensities)
            },
            'imu': {
                'orientation': {
                    'x': imu_msg.orientation.x,
                    'y': imu_msg.orientation.y,
                    'z': imu_msg.orientation.z,
                    'w': imu_msg.orientation.w
                },
                'angular_velocity': {
                    'x': imu_msg.angular_velocity.x,
                    'y': imu_msg.angular_velocity.y,
                    'z': imu_msg.angular_velocity.z
                },
                'acceleration': {
                    'x': imu_msg.linear_acceleration.x,
                    'y': imu_msg.linear_acceleration.y,
                    'z': imu_msg.linear_acceleration.z
                }
            }
        })


def main():
    rclpy.init()
    node = MultiSensorSaver()

    print("Digite 's' e ENTER para gravar ambos os sensores:")
    if input().lower() != 's':
        return

    print("Gravando Lidar + IMU por 10 segundos...")
    node.saving = True

    start_t = node.get_clock().now().nanoseconds / 1e9

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        current_t = node.get_clock().now().nanoseconds / 1e9
        if current_t - start_t > 10:
            break

    node.saving = False

    print(f"Finalizado! {len(node.data_log)} pares sincronizados.")

    with open('telemetria_total.yaml', 'w') as f:
        yaml.dump(node.data_log, f, default_flow_style=False)

    print("Dados salvos em telemetria_total.yaml")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
