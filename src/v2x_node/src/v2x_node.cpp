#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class V2XNode:public rclcpp::Node
{
public:
  V2XNode()
  : Node("V2XNode")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&V2XNode::topic_callback, this, std::placeholders::_1)
    );
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<V2XNode>());
  rclcpp::shutdown();
  return 0;
}
