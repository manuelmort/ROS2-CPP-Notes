#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Inherit Node as our baseclass
class HeartbeatNode : public rclcpp::Node {
	public:
		HeartbeatNode() : rclcpp::Node("hearbeat_node"), count_(0) {
      
      timer_ = this->create_wall_timer(1s, std::bind(&HeartbeatNode::on_timer,this));	 
      RCLCPP_INFO(this->get_logger(), "Heartbeat Node started"); 
    } 

  private:    
    void on_timer() { 
      RCLCPP_INFO(this->get_logger(), "tick %zu", count_++);
    } 
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t count_;
          
};

int main(int argc, char ** argv) {

	rclcpp::init(argc,argv);

	auto node = std::make_shared<HeartbeatNode>();
	rclcpp::spin(node);	

	rclcpp::shutdown();
	return 0;
}
