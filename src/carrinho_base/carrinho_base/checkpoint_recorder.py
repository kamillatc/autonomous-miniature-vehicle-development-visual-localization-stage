import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import time

# Caminho onde o arquivo vai ser salvo
ARQUIVO = '/home/epiibots/ros2_ws/checkpoints.json'

class GravadorCheckpoints(Node):
    def __init__(self):
        super().__init__('checkpoint_recorder')

        # Lista que vai guardar todos os comandos gravados
        self.checkpoints = []

        # Guarda o tempo do último comando para calcular o intervalo
        self.ultimo_tempo = time.time()

        # Assina o /cmd_vel para escutar os comandos do teleop
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.salvar_comando,
            5
        )

        self.get_logger().info('Gravador iniciado! Dirija o carrinho...')
        self.get_logger().info(f'Salvando em: {ARQUIVO}')

    def salvar_comando(self, msg):
        agora = time.time()

        # Calcula quanto tempo passou desde o último comando
        intervalo = agora - self.ultimo_tempo
        self.ultimo_tempo = agora

        # Monta o checkpoint com o comando e o intervalo
        checkpoint = {
            'intervalo': round(intervalo, 3),  # segundos
            'linear_x':  round(msg.linear.x, 3),
            'angular_z': round(msg.angular.z, 3)
        }

        self.checkpoints.append(checkpoint)
        self.get_logger().info(
            f'Gravado: linear={msg.linear.x:.2f} angular={msg.angular.z:.2f} intervalo={intervalo:.3f}s'
        )

    def salvar_arquivo(self):
        # Chamado ao encerrar com Ctrl+C — salva tudo no arquivo
        with open(ARQUIVO, 'w') as f:
            json.dump(self.checkpoints, f, indent=2)
        self.get_logger().info(f'{len(self.checkpoints)} checkpoints salvos em {ARQUIVO}')

def main():
    rclpy.init()
    no = GravadorCheckpoints()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        # Ao apertar Ctrl+C salva o arquivo antes de sair
        no.salvar_arquivo()
    finally:
        no.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
