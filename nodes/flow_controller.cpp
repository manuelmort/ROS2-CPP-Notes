#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;


class FlowController : public rclcpp::Node { 
public:
  FlowController() :  rclcpp::Node("flow_controller"), flow_rate_(1.87f),running_(true) { 

    setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/flow/setpoint",10);
    flowstate_publisher_ = this->create_publisher<std_msgs::msg::String>("/flow/state",10); 
    
    on_timer_ = this->create_wall_timer(100ms,[this]()->void{
      RCLCPP_INFO(this->get_logger(),"Publishing..."); 
      std_msgs::msg::Float32 setpoint_msg; 
      std_msgs::msg::String state_msg; 

      if(running_) {
          setpoint_msg.data = flow_rate_;
          state_msg.data = "RUNNING";

        }
      else {
          setpoint_msg.data = 0.0f;
          state_msg.data = "STOPPED"; 
      }
      this->setpoint_publisher_->publish(setpoint_msg);
      this->flowstate_publisher_->publish(state_msg);
             
    });
  
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr setpoint_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flowstate_publisher_;
  rclcpp::TimerBase::SharedPtr on_timer_;
  float flow_rate_;
  bool running_;
 
};



int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FlowController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

 return 0; 
}
