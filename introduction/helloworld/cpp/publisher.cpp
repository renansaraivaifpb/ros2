#include <chrono>           // std::chrono_literals (ex: 500ms)
#include <memory>           // std::make_shared
#include <string>           // std::to_string

#include "rclcpp/rclcpp.hpp"          // biblioteca principal do ros2 c++
#include "std_msgs/msg/string.hpp"    // tipo de mensagem String

/*
* Publisher example that periodically sends out a string
*/

using namespace std::chrono_literals;

class CppTalker : public rclcpp::Node{
  public:
    // construtor onde é inicializado o ó
    CppTalker() : Node("Publisher"), // nome do nó
                  counter_(0)          // inicializa o contador
      {
        // objeto Publisher 
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic",10);

        // timer
        // é usado std::bind para ligar a função de membro 'this'
        timer_ = this->create_wall_timer(500ms, std::bind(&CppTalker::timer_callback, this));
      };
  private:
      // declaração dos membros da classe 
      // ponteiro compartilhado. Ajuda evitar vazamentos de memória e ponteiros pendentes
      // permitindo que tenha múltiplos proprietários. 
      rclcpp::TimerBase::SharedPtr timer_; // ponteiro inteligente para o timer
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // ponteiro para o publisher
      size_t counter_ = 0; // contador de mensagens

      // callback chamada pelo timer
      void timer_callback(){
        // cria a mensagem
        auto message = std_msgs::msg::String();
        // define o conteudo
        message.data = "Hello from C++: " + std::to_string(counter_++);
        // loga no console (usando a macro de loggin do ros2)
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // publica a mensagem
        publisher_->publish(message);

      };

};


/*
 * Main entrypoint
*/
int main(int argc, char * argv[]){
  // inicializa ros2
  rclcpp::init(argc, argv);
  // inicializa e executa o nó
  auto node = std::make_shared<CppTalker>();
  // gira (spin) o nó, o que processa os callbacks
  // instância do nó CppTalker e é passado para o spin
  rclcpp::spin(node);

  // desliga o ROS2 ao sair (ex: Ctrl+c)
  rclcpp::shutdown();

  return 0;
}
