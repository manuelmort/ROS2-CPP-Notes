#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class HeartbeatSub : public rclcpp::Node
{
public:
  HeartbeatSub() : rclcpp::Node("subscriber_node")
  {
    auto receiver_callback =
      [this](const std_msgs::msg::String & msg) {
        RCLCPP_INFO(this->get_logger(),
                    "Received %s", msg.data.c_str());
      };

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, receiver_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartbeatSub>());
  rclcpp::shutdown();
  return 0;
}

