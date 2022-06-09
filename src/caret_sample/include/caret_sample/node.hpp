
#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace Node {

static constexpr int QOS_HISTORY_SIZE = 10;

class NodePub : public rclcpp::Node
{
public:
  NodePub(std::string node_name, std::string pub_topic_name, int period_ms = 10, int latency_ms = 1)
  : Node(node_name), count_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),[=]()
      {
        rclcpp::sleep_for(std::chrono::milliseconds(latency_ms));
        auto msg = std_msgs::msg::Int32();
        msg.data = count_++;
        pub_->publish(msg);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int count_;
};

class NodeSub : public rclcpp::Node
{
public:
  NodeSub(std::string node_name, std::string sub_topic_name, int latency_ms = 2)
  : Node(node_name)
  {
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(latency_ms));
        RCLCPP_INFO(this->get_logger(), "data = %d", msg->data);
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};


class NodeSubPub : public rclcpp::Node
{
public:
  NodeSubPub(std::string node_name, std::string sub_topic_name, std::string pub_topic_name, int latency_ms = 2)
  : Node(node_name)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(std::chrono::milliseconds(latency_ms));
        auto msg_pub = std_msgs::msg::Int32();
        msg_pub.data = msg->data;
        pub_->publish(msg_pub);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

}