# Pacote ROS 2 Pub/Sub Básico em Python (`my_pkg_py`)

Este pacote é uma implementação introdutória do padrão "Publisher/Subscriber" (Publicador/Assinante) no ROS 2, utilizando a biblioteca `rclpy` (Python).

Ele contém dois nós executáveis:
* `PyTalker`: Um nó publicador que envia uma mensagem `std_msgs/String` a cada 0.5 segundos no tópico `my_topic`.
* `PySub`: Um nó assinante que escuta o tópico `my_topic` e imprime qualquer mensagem recebida no console.

## Arquitetura dos Nós (`rqt_graph`)

A imagem abaixo, gerada com `rqt_graph`, mostra os nós em execução e como eles se comunicam através do tópico `/my_topic`.

![Gráfico RQT dos Nós](https://raw.githubusercontent.com/renansaraivaifpb/ros2/refs/heads/main/introduction/helloworld/ros_introduction.png)
