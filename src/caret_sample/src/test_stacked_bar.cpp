#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "caret_sample/node.hpp"


class NodeSubPubHugeLatency : public rclcpp::Node
{
public:
  NodeSubPubHugeLatency(std::string node_name, std::string sub_topic_name, std::string pub_topic_name, int latency_ms = 25)
  : Node(node_name), cnt_(0)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, 1);
    sub_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name, 1,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(1 * 1e6)));
        if (cnt_++ % 40 == 0) {
        // if (cnt_++ == 80) {
          rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(latency_ms * 1e6)));
        }
        auto msg_pub = std_msgs::msg::Int32();
        msg_pub.data = msg->data;
        pub_->publish(msg_pub);
        RCLCPP_INFO(this->get_logger(), "data = %d", msg->data);
      });
  }

private:
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  int cnt_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::vector<std::shared_ptr<rclcpp::Node>> node_list;

  node_list.emplace_back(std::make_shared<SampleNode::NodePub>("node_src", "/topic_src"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("node_1", "/topic_src", "/topic_1"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("node_2", "/topic_1", "/topic_2"));
  node_list.emplace_back(std::make_shared<NodeSubPubHugeLatency>("node_3", "/topic_2", "/topic_3"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("node_dst", "/topic_3", "/topic_4"));

  for (auto & node : node_list) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}
