#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "caret_sample/node.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::vector<std::shared_ptr<rclcpp::Node>> node_list;

  node_list.emplace_back(std::make_shared<SampleNode::NodePub>("node_src", "/topic_src", 10, 2));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("node_1", "/topic_src", "/topic_1"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("node_2", "/topic_1", "/topic_2"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSubPub>("node_3", "/topic_2", "/topic_dst"));
  node_list.emplace_back(std::make_shared<SampleNode::NodeSub>("node_dst", "/topic_dst", 4));

  for (auto & node : node_list) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}
