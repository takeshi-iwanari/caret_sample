
#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

namespace SampleNode {

static constexpr int QOS_HISTORY_SIZE = 10;

std::chrono::nanoseconds make_jitter(int time_ms, double jitter = 0.2)
{
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  std::uniform_real_distribution<> dist(time_ms * (1.0 - jitter), time_ms * (1.0 + jitter));
  double time_jitter_ms = dist(engine);
  return std::chrono::nanoseconds(static_cast<int>(time_jitter_ms * 1e6));
}

class NodePub : public rclcpp::Node
{
public:
  NodePub(std::string node_name, std::string pub_topic_name, int period_ms = 10, int latency_ms = 1)
  : Node(node_name), count_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_ms));
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
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
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
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        auto msg_pub = std_msgs::msg::Int32();
        msg_pub.data = msg->data;
        pub_->publish(msg_pub);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

class NodeSubStorePub : public rclcpp::Node
{
public:
  NodeSubStorePub(std::string node_name, std::string sub_topic_name, std::string pub_topic_name, int num_store = 3, int latency_ms = 2)
  : Node(node_name), num_store_(num_store), count_store_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        count_store_++;
        if (count_store_ == num_store_) {
          count_store_ = 0;
          auto msg_pub = std_msgs::msg::Int32();
          msg_pub.data = msg->data;
          pub_->publish(msg_pub);
        }
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  const int num_store_;
  int count_store_;
};

class NodeSubStorePubTimer : public rclcpp::Node
{
public:
  NodeSubStorePubTimer(std::string node_name, std::string sub_topic_name, std::string pub_topic_name, int period_ms = 10, int latency_ms = 2, int latency_timer_ms = 1)
  : Node(node_name), period_ms_(period_ms), count_store_(0), last_data_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        count_store_++;
        last_data_ = msg->data;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_timer_ms, 0.2));
        auto msg = std_msgs::msg::Int32();
        msg.data = last_data_;
        count_store_ = 0;
        last_data_ = 0;
        pub_->publish(msg);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  const int period_ms_;
  int count_store_;
  int last_data_;
};

class NodeSub3Pub1 : public rclcpp::Node
{
public:
  NodeSub3Pub1(std::string node_name, std::string sub_topic_name_0, std::string sub_topic_name_1, std::string sub_topic_name_2, std::string pub_topic_name)
  : Node(node_name), stored_data_num_(0), stored_data_sum_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);

    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_0 = std::bind(&NodeSub3Pub1::topic_callback, this, std::placeholders::_1, 0);
    sub_0_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_0, QOS_HISTORY_SIZE, cb_0);
    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_1 = std::bind(&NodeSub3Pub1::topic_callback, this, std::placeholders::_1, 1);
    sub_1_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_1, QOS_HISTORY_SIZE, cb_1);
    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_2 = std::bind(&NodeSub3Pub1::topic_callback, this, std::placeholders::_1, 2);
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


class NodeSub1PubTimer3 : public rclcpp::Node
{
public:
  NodeSub1PubTimer3(std::string node_name, std::string sub_topic_name, std::string pub_topic_name_0, std::string pub_topic_name_1, std::string pub_topic_name_2,
    int period_ms_0 = 100, int period_ms_1 = 100, int period_ms_2 = 100, int latency_ms = 2, int latency_timer_ms = 1)
  : Node(node_name), period_ms_0_(period_ms_0), period_ms_1_(period_ms_1), period_ms_2_(period_ms_2), last_data_(0)
  {
    pub_0_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name_0, QOS_HISTORY_SIZE);
    pub_1_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name_1, QOS_HISTORY_SIZE);
    pub_2_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name_2, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        last_data_ = msg->data;
      });
    timer_0_ = create_wall_timer(std::chrono::milliseconds(period_ms_0_),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_timer_ms, 0.2));
        auto msg = std_msgs::msg::Int32();
        msg.data = last_data_;
        pub_0_->publish(msg);
      });
    timer_1_ = create_wall_timer(std::chrono::milliseconds(period_ms_1_),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_timer_ms, 0.2));
        auto msg = std_msgs::msg::Int32();
        msg.data = last_data_;
        pub_1_->publish(msg);
      });
    timer_2_ = create_wall_timer(std::chrono::milliseconds(period_ms_2_),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_timer_ms, 0.2));
        auto msg = std_msgs::msg::Int32();
        msg.data = last_data_;
        pub_2_->publish(msg);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_0_, pub_1_, pub_2_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_0_, timer_1_, timer_2_;
  const int period_ms_0_, period_ms_1_, period_ms_2_;
  int last_data_;
};

class NodeFeedbackFormer : public rclcpp::Node
{
public:
  NodeFeedbackFormer(std::string node_name, std::string sub_topic_name, std::string sub_feedback_topic_name, std::string pub_topic_name, int latency_ms = 2)
  : Node(node_name), data_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        auto msg_pub = std_msgs::msg::Int32();
        msg_pub.data = (int)((msg->data + data_) / 2);
        pub_->publish(msg_pub);
      });
    sub_feedback_ = create_subscription<std_msgs::msg::Int32>(sub_feedback_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        data_ = msg->data;
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_feedback_;
  int data_;
};

class NodeFeedbackLatter : public rclcpp::Node
{
public:
  NodeFeedbackLatter(std::string node_name, std::string sub_topic_name, std::string pub_topic_name, std::string pub_feedback_topic_name, int latency_ms = 2)
  : Node(node_name), data_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    pub_feedback_ = create_publisher<std_msgs::msg::Int32>(pub_feedback_topic_name, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        auto msg_pub = std_msgs::msg::Int32();
        msg_pub.data = msg->data;
        pub_->publish(msg_pub);
        
        auto msg_pub_feedback = std_msgs::msg::Int32();
        msg_pub_feedback.data = msg->data;
        pub_feedback_->publish(msg_pub_feedback);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_feedback_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_feedback_;
  int data_;
};

class NodeSubStorePubTimer2 : public rclcpp::Node
{
public:
  NodeSubStorePubTimer2(std::string node_name, std::string sub_topic_name, std::string pub_topic_name, int latency_ms = 2, int period_timer_ms_1 = 50, int latency_timer_ms_1 = 1, int period_timer_ms_2 = 20, int latency_timer_ms_2 = 2)
  : Node(node_name), data_src_(0), data_timer_1_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(make_jitter(latency_ms, 0.2));
        data_src_ = msg->data;
      });
    timer_1_ = create_wall_timer(std::chrono::milliseconds(period_timer_ms_1),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_timer_ms_1, 0.2));
        data_timer_1_ = data_src_ * 2;
      });
    timer_2_ = create_wall_timer(std::chrono::milliseconds(period_timer_ms_2),[=]()
      {
        rclcpp::sleep_for(make_jitter(latency_timer_ms_2, 0.2));
        auto msg = std_msgs::msg::Int32();
        msg.data = data_timer_1_ * 2;
        pub_->publish(msg);

      });

  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_1_, timer_2_;
  int data_src_;
  int data_timer_1_;
};

}