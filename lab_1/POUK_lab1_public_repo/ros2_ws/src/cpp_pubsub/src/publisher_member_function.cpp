//подключение стандартных библиотек
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" // позволяет пользоваться наиболее распространенными частями ROS
#include "std_msgs/msg/string.hpp" // встроенный тип сообщения, для публикации данных

using namespace std::chrono_literals;

// создаем класс MinimalPublisher путем наследования от rclcpp::Node
class MinimalPublisher : public rclcpp::Node{
  public:

  // конструктор класса, присваиваем ноду имя "minimal_publisher" и count_=0
  MinimalPublisher() : Node("minimal_publisher"), count_(0)
  {
    // инициализируем publisher_, он будет отправлять строковый тип сообщения из "topic1", также в его параметрах
    // задается лимит очереди для ограничения количества сообщений 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic1", 10);
    
    // инициализируем timer_, выполненяет функцию timer_callback раз в 500ms.
    // std::bind создаёт объект-функцию, который привязывает метод timer_callback к текущему объекту (this) и 
    // позволяет вызывать этот метод как обычную функцию, без необходимости явно передавать объект.
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));

    // Создаем Subscriber, который будет подписываться на топик "topic2" и получать сообщения типа String
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic2", 10, std::bind(&MinimalPublisher::topic_callback, this, std::placeholders::_1));  
  }
  private:
    void timer_callback()
    {
      // Создаем сообщение типа String
      auto message = std_msgs::msg::String();
      message.data = "Hello world! " + std::to_string(count_++);

      // Выводим сообщение на экран
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      
      // Публикуем сообщение в топик
      publisher_->publish(message);
    }
    
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      // Выводим полученное из "topic2" сообщение на экран
      RCLCPP_INFO(this->get_logger(), "PONIL: '%s'", msg->data.c_str());
    }
    
    // объявляем поля класса
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};


int main(int args, char * argv[])
{
  // Инициализация ROS2
  rclcpp::init(args, argv);

  // Создаем и запускаем ноду
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  // Завершаем работу ROS2
  rclcpp::shutdown();
  return 0;
}
