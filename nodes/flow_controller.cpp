#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;


class FlowController : public rclcpp::Node { 
public:
  FlowController() :  rclcpp::Node("flow_controller"), count_(0) {

    setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float32>("flow/setpoint",10);
    flowstate_publisher_ = this->create_publisher<std_msgs::msg::String>("flow/state",10); 

  }
private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flowstate_publisher_;
  size_t count_;
};



int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlowController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

 return 0; 
}
