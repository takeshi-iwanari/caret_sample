#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "caret_sample/node.hpp"


class NodeSub2Pub1 : public rclcpp::Node
{
public:
  NodeSub2Pub1(std::string node_name, std::string sub_topic_name_0, std::string sub_topic_name_1, std::string pub_topic_name, int latency_ms = 1)
  : Node(node_name)
  {
    static constexpr int QOS_HISTORY_SIZE = 10;
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);
    sub_0_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_0, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(latency_ms * 1e6)));
        RCLCPP_INFO(this->get_logger(), "data = %d", msg->data);
        pub_->publish(*msg);
      });
    sub_1_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_1, QOS_HISTORY_SIZE,
      [=](std_msgs::msg::Int32::UniquePtr msg)
      {
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(latency_ms * 1e6)));
        RCLCPP_INFO(this->get_logger(), "data = %d", msg->data);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_0_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_1_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::vector<std::shared_ptr<rclcpp::Node>> node_list;

  node_list.emplace_back(std::make_shared<SampleNode::NodePub>("Node_A", "/topic_A"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("Node_B", "/topic_A", "/topic_B", 2));
  node_list.emplace_back(std::make_shared<SampleNode::NodePub>("Node_AA", "/topic_AA"));
  node_list.emplace_back(std::make_shared<NodeSub2Pub1>("Node_C", "/topic_AA", "/topic_B", "/topic_C"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("Node_D", "/topic_C", "/topic_D"));
  node_list.emplace_back(std::make_shared<SampleNode::NodePub>("Node_DD", "/topic_DD", 2));
  node_list.emplace_back(std::make_shared<NodeSub2Pub1>("Node_E", "/topic_DD", "/topic_D", "/topic_E"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSub>("Node_F", "/topic_E", 1));


  for (auto & node : node_list) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}
