import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import random

from example_interfaces.srv import AddTwoInts

class PyClient(Node):

    def __init__(self):
        """construtor"""
        super().__init__("add_ints_client")

        self._client = self.create_client(
            AddTwoInts,
            'add_ints'
        )

        # esperar o servidor ficar disponível
        while not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('[Server] - indisponível')
        
        # chama o metodo periodicamente
        self._timer = self.create_timer(2.0, self.send_request)

    def send_request(self):

        # preenche a mensagem de requuest
        req = AddTwoInts.Request()
        req.a = random.randint(0, 10)
        req.b = random.randint(0, 10)
    

        # envia chamada de serviço de forma assíncrona
        # retornando objeto future que é uma "promessa"
        # de que a resposta chegara eventualmente
        self.future = self._client.call_async(req)
        self.future.add_done_callback(self._response_callback)
        self.get_logger().info(f"Enviando: {req.a} + {req.b}")


        # retorna future para que a funcao main possa esperar por ele
        return self.future

    def _response_callback(self, future):

        try:
            resp = future.result()
            self.get_logger().info(f"Result: {resp.sum}")
        except Exception as e:
            self.get_logger().error(f"{str(e)}")

def main(args=None):
    """
    Função que inicializa e executa o nó server
    """
    node = None
    try:
        # inicializa a library rclpy
        rclpy.init(args=args)

        # instancia o nó servidor
        node = PyClient()

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
