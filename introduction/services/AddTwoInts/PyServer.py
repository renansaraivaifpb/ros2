# Bibliotecas
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

# Classe do nó
class PyServer(Node):
    """
    Servidor que reage a requisições, somando (a + b) e retornando o resultado (sum)
    """

    def __init__(self):
        """
        Construtor da classe
        """
        #inicializa a classe base e atribui nome ao nó
        super().__init__('py_server')

        # cria o serviço (server)
        self.src = self.create_service(AddTwoInts, 'add_ints', self.callback)

        self.get_logger().info('[Server]: status - OK')

    def callback(self, request, response):
        """
        é executado sempre que o serviço add_two_ints for chamado

        :param request: contem .a e .b
        :param response: contem .sum
        """

        response.sum = request.a + request.b

        # operação de debug no console
        self.get_logger().info(f'Requisição recebido: {request.a} + {request.b} = {response.sum}')

        return response


# função principal (entry point)

def main(args=None):
    """
    Função que inicializa e executa o nó server
    """
    node = None
    try:
        # inicializa a library rclpy
        rclpy.init(args=args)

        # instancia o nó servidor
        node = PyServer()

        # gira o nó (spin), fazendo com o que o server fique em espera
        rclpy.spin(node)

    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# ponto de execução padrão
if __name__ == "__main__":
    main()
