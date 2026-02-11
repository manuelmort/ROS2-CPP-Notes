#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;


class HeartbeatPub : public rclcpp::Node {
public: 
  HeartbeatPub() : rclcpp::Node("heartbeat_publisher"), count_(0) {
  
    //Creating a publisher object
    setpoint_publisher_ = this->create_publisher<std_msgs::msg::Float32> ("flow/setpoint",10);
    flowstate_publisher_ = this->create_publisher<std_msgs::msg::String>("flow/state",10);

    auto flow_callback = [this]() -> void {

        auto message = std_msgs::msg::String(); 
        RCLCPP_INFO(this->get_logger(), "Flow rate: 1.87mL/min");
        
        this->flow_publisher_->publish(message);
     };

    //Creating wall clock timer
    timer_ = this->create_wall_timer(0.1s, on_timer());
      
    auto on_shutdown_timer = [this]() -> void { 
      RCLCPP_INFO(this->get_logger(), "Shutting down Node");
      rclcpp::shutdown();
    };
   
    //creating shutdown timer
    shutdown_timer_ = this->create_wall_timer(10s,on_shutdown_timer); 
  }
private:
 
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr flowstate_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr setpoint_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  std::size_t count_;

};

int main(int argc, char **argv) {
  
  rclcpp::init(argc, argv);  
  auto node = std::make_shared<HeartbeatPub>(); 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
