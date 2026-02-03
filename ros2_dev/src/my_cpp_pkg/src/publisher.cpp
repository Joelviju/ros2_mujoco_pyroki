#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode() : Node("publisher_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&PublisherNode::publish_message, this));
  }

private:
  void publish_message()
  {
    std_msgs::msg::String msg;
    msg.data = "Hello from ROS 2 C++ publisher";
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}
