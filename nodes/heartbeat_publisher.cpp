#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;


class HeartbeatPub : public rclcpp::Node {
public: 
  HeartbeatPub() : rclcpp::Node("heartbeat_publisher"), count_(0) {

    //Creating a publisher 
    publisher_ = this->create_publisher<std_msgs::msg::String>("heartbeat",10);
    //Creating wall clock timer
    timer_ = this->create_wall_timer(1s, std::bind(&HeartbeatPub::on_timer,this));
     
    RCLCPP_INFO(this->get_logger(),"Heartbeat started");
  }

private:
 
  void on_timer() {
    std_msgs::msg::String msg;
    RCLCPP_INFO(this->get_logger(),"tick: %zu", count_++); 
    
    publisher_->publish(msg);
  } 
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::size_t count_;

};

int main(int argc, char **argv) {
  
  rclcpp::init(argc, argv);  
  auto node = std::make_shared<HeartbeatPub>(); 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
