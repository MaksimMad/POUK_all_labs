#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PatrolBot : public rclcpp::Node {
public:
  PatrolBot() : Node("patrol_bot_node"), count_(0), current_goal_index_(-1), goal_active_(false) {
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10);

    point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&PatrolBot::clickPointCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PatrolBot::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PatrolBot::timerCallback, this));

    this->declare_parameter("goal_tolerance", 0.3);
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  }

private:
  void timerCallback() {
    if (!goal_active_ && count_ > 0) {
      sendNextGoal();
    }
  }

  void sendNextGoal() {
    if (count_ == 0) return;

    current_goal_index_ = (current_goal_index_ + 1) % count_;
    auto msg = goals_[current_goal_index_];
    msg.header.stamp = this->now();
    goal_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "Sending goal %d: [%.2f, %.2f]", 
               current_goal_index_,
               msg.pose.position.x,
               msg.pose.position.y);
    
    goal_active_ = true;
    goal_start_time_ = this->now();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!goal_active_ || count_ == 0) return;

    auto current_goal = goals_[current_goal_index_];
    double dx = msg->pose.pose.position.x - current_goal.pose.position.x;
    double dy = msg->pose.pose.position.y - current_goal.pose.position.y;
    double distance = sqrt(dx*dx + dy*dy);

    auto elapsed = this->now() - goal_start_time_;
    if (elapsed.seconds() > 10.0) {
      RCLCPP_WARN(this->get_logger(), "Goal timeout, moving to next");
      goal_active_ = false;
      return;
    }

    if (distance < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal %d reached!", current_goal_index_);
      goal_active_ = false;
    }
  }

  void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point) {
    if (count_ >= 5) {
      RCLCPP_WARN(this->get_logger(), "Maximum number of points (5) reached");
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.pose.position = point->point;
    
    // Ручное задание ориентации (90 градусов)
    double target_angle = M_PI/2;
    goal_pose.pose.orientation.z = sin(target_angle/2);
    goal_pose.pose.orientation.w = cos(target_angle/2);
    
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->now();

    goals_[count_++] = goal_pose;

    RCLCPP_INFO(this->get_logger(), "Added point %d: [%.2f, %.2f]", 
               count_-1, point->point.x, point->point.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped goals_[5];
  int count_;
  int current_goal_index_;
  bool goal_active_;
  rclcpp::Time goal_start_time_;
  double goal_tolerance_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolBot>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}