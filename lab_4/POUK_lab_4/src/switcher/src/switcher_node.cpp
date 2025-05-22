#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"

#include <iostream>
#include <string>

using namespace std;

class Switcher : public rclcpp::Node{
    public:
    Switcher() : Node("switcher"){

        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("/selector", 10);
        printMenu();
        input();
    }


    private:
    void printMenu(){
        cout << "0 - Dummy\n";
        cout << "1 - Voyager\n";
        cout << "2 - Wall\n";
    }

    void input(){
        int select;
        while (rclcpp::ok())
        {
            cin>>select;

            if(select >= 0 && select <=2){
                std_msgs::msg::UInt16 msg = std_msgs::msg::UInt16();
                msg.data = select;
                publisher_->publish(msg);
            }
            else{
                cout << "Invalid input\n";
            }
            
            printMenu();
        }
        
    }

    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
    
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Switcher>());
    rclcpp::shutdown();
    
    return 0;
}