#include <chrono>           // std::chrono_literals (ex: 500ms)
#include <memory>           // std::make_shared
#include <string>           // std::to_string

#include "rclcpp/rclcpp.hpp"          // biblioteca principal do ros2 c++
#include "std_msgs/msg/string.hpp"    // tipo de mensagem String


// usar std::placeholders::_1 permite "reservar" o espaço para o argumento da
// mensagem que o callback receberá.
using std::placeholders::_1;

class CppSubscriber : public rclcpp::Node{
  public:
    // construtor
    CppSubscriber() : Node("subscriber"){
      // cria o assinante
      //    -my_topic
      //    -QoS (buffer): 10
      //    igual ao publisher, std:bind é necessário para dizer à 'create_subscription'
      //    para chamar a função 'topic_callback' deste objeto ('this') e passar um 
      //    argumento representado por '_1'
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "my_topic",
        10,
        std::bind(
          &CppSubscriber::listener_callback,
          this,
          _1
        )
      );

    };

  private:
    // declaracao de variáveis 
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void listener_callback(const std_msgs::msg::String & msg){
      RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg.data.c_str());
    };
};


int main(int argc, char * argv[]){
  // inicializa o ros2
  rclcpp::init(argc, argv);
  // inicialia e executa o node
  auto node = std::make_shared<CppSubscriber>();
  rclcpp::spin(node);
  // cleanup
  rclcpp::shutdown();

  return 0;
}
