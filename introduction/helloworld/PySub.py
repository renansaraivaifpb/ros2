# --- Importações de Bibliotecas ---
import rclpy
from rclpy.node import Node  # Classe base para criar um nó
from rclpy.executors import ExternalShutdownException  # Exceção para desligamento
from std_msgs.msg import String  # Tipo de mensagem padrão para texto

# --- Definição da Classe do Nó ---

class PySub(Node):
    """
    Este nó (PySub) se inscreve (subscribe) no tópico 'my_topic'.
    Ele ouve mensagens do tipo String e as imprime no console
    sempre que uma mensagem é recebida.
    """

    def __init__(self):
        """
        Construtor da classe PySub.
        Configura o 'subscriber'.
        """
        # 1. Inicializa a classe base (Node) e dá um nome ao nó
        #    (Por convenção, nomes de nós são em minúsculo)
        super().__init__('py_sub')

        # 2. Cria um 'subscriber' (inscrito)
        #    - Tipo da mensagem: String
        #    - Nome do tópico: 'my_topic' (DEVE ser o mesmo do publicador)
        #    - Callback: self._listener_callback (a função a ser executada)
        #    - QoS (Qualidade de Serviço): Profundidade da fila de 10
        self.subscription = self.create_subscription(
            String,
            "my_topic",
            self._listener_callback,
            10)
        
        # Nota: Mesmo que 'self.subscription' não seja usado
        # explicitamente, é importante mantê-lo como um atributo
        # da classe para evitar que seja coletado pelo "garbage collector".

    def _listener_callback(self, msg):
        """
        Callback executada automaticamente sempre que uma nova
        mensagem chega no tópico 'my_topic'.
        
        :param msg: O objeto da mensagem (tipo String) que foi recebida.
        """
        # 1. Loga a informação recebida no console do ROS2
        #    Usamos '.data' para acessar o conteúdo de uma mensagem String.
        self.get_logger().info(f'Eu ouvi: "{msg.data}"')

# --- Função Principal (Entry Point) ---

def main(args=None):
    """
    Função principal que inicializa e executa o nó 'subscriber'.
    """
    # Inicializa 'node' como None para um desligamento limpo
    # caso a inicialização do nó falhe.
    node = None
    try:
        # 1. Inicializa a biblioteca rclpy
        rclpy.init(args=args)

        # 2. Instancia (cria) o nó 'subscriber'
        node = PySub()

        # 3. "Gira" o nó (spin). Para um subscriber, isso é crucial.
        #    O 'spin' bloqueia o script e fica "esperando" por
        #    eventos (como a chegada de mensagens). Qualquer mensagem
        #    recebida irá disparar o callback '_listener_callback'.
        rclpy.spin(node)

    except(KeyboardInterrupt, ExternalShutdownException):
        # 4. Captura um desligamento (Ctrl+C ou sinal do ROS)
        pass  # A limpeza será feita no 'finally'

    finally:
        # 5. Bloco de finalização: sempre executado
        
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
