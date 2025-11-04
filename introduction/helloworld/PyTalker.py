# --- Importações de Bibliotecas ---
import rclpy
from rclpy.node import Node  # Classe base para criar um nó
from rclpy.executors import ExternalShutdownException  # Exceção para desligamento
from std_msgs.msg import String  # Tipo de mensagem padrão para texto

# --- Definição da Classe do Nó ---

class PyTalker(Node):
    """
    Este nó (PyTalker) publica uma mensagem do tipo String no tópico 
    'my_topic' a uma taxa fixa (definida pelo timer).
    """

    def __init__(self):
        """
        Construtor da classe PyTalker.
        Configura o publicador, o timer e o contador.
        """
        # 1. Inicializa a classe base (Node) e dá um nome ao nó
        super().__init__('py_talker')

        # 2. Cria um publicador (Publisher)
        #    - Tipo da mensagem: String
        #    - Nome do tópico: 'my_topic'
        #    - QoS (Qualidade de Serviço): Profundidade da fila de 10
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)

        # 3. Configura o timer
        timer_period = 0.5  # Período em segundos (equivale a 2 Hz)
        
        # O timer chama self.timer_callback a cada 'timer_period' segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 4. Inicializa um contador interno para a mensagem
        self._counter = 0

    def timer_callback(self):
        """
        Callback executada pelo timer.
        Esta função cria, preenche e publica a mensagem.
        """
        # 1. Cria uma nova instância da mensagem
        msg = String()
        
        # 2. Define o conteúdo da mensagem
        msg.data = f"Hello world: {self._counter}"

        # 3. Publica a mensagem no tópico
        self.publisher_.publish(msg)

        # 4. Loga a ação no console do ROS2 (nível INFO)
        self.get_logger().info(f'Publicando: "{msg.data}"')
        
        # 5. Incrementa o contador
        self._counter += 1

# --- Função Principal (Entry Point) ---

def main(args=None):
    """
    Função principal que inicializa e executa o nó.
    """
    # Inicializa 'node' como None para garantir que o 'finally' 
    # não tente destruir um nó que falhou na criação.
    node = None
    try:
        # 1. Inicializa a biblioteca rclpy
        rclpy.init(args=args)

        # 2. Instancia (cria) o nó
        node = PyTalker()

        # 3. "Gira" o nó (spin), o que o mantém vivo e processando
        #    callbacks (como o timer). O script fica bloqueado aqui
        #    até que o nó receba um sinal de desligamento.
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        # 4. Captura um desligamento iniciado pelo usuário (Ctrl+C)
        #    ou pelo próprio sistema ROS.
        pass  # A limpeza será feita no 'finally'

    finally:
        # 5. Bloco de finalização: sempre executado,
        #    garantindo que os recursos sejam liberados.
        
        # Destrói o nó (se ele foi criado com sucesso)
        if node is not None:
            node.destroy_node()
        
        # Desliga o rclpy (se ainda estiver ativo)
        if rclpy.ok():
            rclpy.shutdown()

# --- Ponto de Execução Padrão do Python ---

if __name__ == "__main__":
    # Se este script for executado diretamente, chama a função main()
    main()
