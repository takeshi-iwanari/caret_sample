
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

class NodeSub3Pub : public rclcpp::Node
{
public:
  NodeSub3Pub(std::string node_name, std::string sub_topic_name_0, std::string sub_topic_name_1, std::string sub_topic_name_2, std::string pub_topic_name)
  : Node(node_name), stored_data_num_(0), stored_data_sum_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);

    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_0 = std::bind(&NodeSub3Pub::topic_callback, this, std::placeholders::_1, 0);
    sub_0_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_0, QOS_HISTORY_SIZE, cb_0);
    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_1 = std::bind(&NodeSub3Pub::topic_callback, this, std::placeholders::_1, 1);
    sub_1_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_1, QOS_HISTORY_SIZE, cb_1);
    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_2 = std::bind(&NodeSub3Pub::topic_callback, this, std::placeholders::_1, 2);
    sub_2_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_2, QOS_HISTORY_SIZE, cb_2);
  }

private:
  void topic_callback(const std_msgs::msg::Int32::UniquePtr msg, int index)
  {
    (void)index;
    stored_data_num_++;
    stored_data_sum_ += msg->data;
    if (stored_data_num_ == 3) {
    // if (index == 2) {
        auto msg_pub = std_msgs::msg::Int32();
        msg_pub.data = stored_data_sum_;
        pub_->publish(msg_pub);
        stored_data_num_ = 0;
        stored_data_sum_ = 0;
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_0_, sub_1_, sub_2_;
  int stored_data_num_;
  int stored_data_sum_;
};

}