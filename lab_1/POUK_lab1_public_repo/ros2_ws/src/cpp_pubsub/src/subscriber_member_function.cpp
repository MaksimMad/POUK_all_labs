#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

// создаем класс MinimalSubscriber путем наследования от rclcpp::Node
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // конструктор класса, присваиваем ноду имя "minimal_subscriber" и count_=0
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      // Создаем Subscriber, который будет подписываться на топик "topic1" и получать сообщения типа String
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic1", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      // Выводим полученное из "topic1" сообщение на экран
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

      RCLCPP_INFO(this->get_logger(), "I say GOODBAY");

      // инициализируем publisher_, он будет отправлять строковый тип сообщения из "topic2", также в его параметрах
      // задается лимит очереди для ограничения количества сообщений 10
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic2", 10);

      // Создаем сообщение типа String
      auto message = std_msgs::msg::String();

      // достаем числовое значение из полученного сообщения
      sscanf(msg->data.c_str(), "%*[^0-9]%ld", &count_);
     
      count_ += 1;

      message.data = "GOODBAY talker " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), message.data);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // Публикуем сообщение в топик
      publisher_->publish(message);
    }

    // объявляем поля класса
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    long int count_ = -100;
};

int main(int argc, char * argv[])
{
  // Инициализация ROS2
  rclcpp::init(argc, argv);

  // Создаем и запускаем ноду
  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  // Завершаем работу ROS2
  rclcpp::shutdown();
  return 0;
}
