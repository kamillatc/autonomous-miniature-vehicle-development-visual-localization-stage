import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import os

# Mesmo arquivo que o gravador salvou
ARQUIVO = '/home/epiibots/ros2_ws/checkpoints.json'

class PlayerCheckpoints(Node):
    def __init__(self):
        super().__init__('checkpoint_player')

        # Publisher que vai enviar os comandos para o carrinho
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 5)

        # Inicializa variáveis de controle
        self.checkpoints = []
        self.indice_atual = 0
        self.timer = None

        self.get_logger().info('Player iniciado! Carregando percurso...')
        self.carregar_e_iniciar()

    def carregar_e_iniciar(self):
        # Verifica se o arquivo existe antes de abrir
        if not os.path.exists(ARQUIVO):
            self.get_logger().error(f'Arquivo não encontrado: {ARQUIVO}')
            return

        with open(ARQUIVO, 'r') as f:
            self.checkpoints = json.load(f)

        self.get_logger().info(f'{len(self.checkpoints)} checkpoints carregados')

        # Inicia a execução do primeiro checkpoint se a lista não estiver vazia
        if self.checkpoints:
            self.executar_proximo()
        else:
            self.get_logger().warn('O arquivo de checkpoints está vazio.')

    def executar_proximo(self):
        # Cancela o timer anterior para não acumular loops
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)

        # Se chegou ao fim da lista, para o carrinho
        if self.indice_atual >= len(self.checkpoints):
            self.parar()
            return

        # Pega os dados do checkpoint atual
        cp = self.checkpoints[self.indice_atual]
        
        # Monta e publica a mensagem
        msg = Twist()
        msg.linear.x = float(cp['linear_x'])
        msg.angular.z = float(cp['angular_z'])
        self.publisher.publish(msg)

        self.get_logger().info(
            f'[{self.indice_atual + 1}/{len(self.checkpoints)}] '
            f'linear={msg.linear.x:.2f} angular={msg.angular.z:.2f} por {cp["intervalo"]:.2f}s'
        )

        # Prepara o índice para o próximo passo
        self.indice_atual += 1

        # Cria um timer dinâmico com o tempo do 'intervalo' para chamar esta mesma função novamente
        self.timer = self.create_timer(float(cp['intervalo']), self.executar_proximo)

    def parar(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Percurso concluído! Carrinho parado.')
        
        # Opcional: Desliga o nó automaticamente após o término
        # rclpy.shutdown()

def main():
    rclpy.init()
    no = PlayerCheckpoints()
    
    try:
        # O spin mantém o nó vivo e processando os timers em background
        rclpy.spin(no)
    except KeyboardInterrupt:
        # Se o usuário apertar Ctrl+C, tenta parar o carrinho por segurança
        no.parar()
    finally:
        no.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
