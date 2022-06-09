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

  node_list.emplace_back(std::make_shared<Node::NodePub>("node_0", "/topic0"));
  node_list.emplace_back(std::make_shared<Node::NodeSubPub>("node_1", "/topic0", "/topic1"));
  node_list.emplace_back(std::make_shared<Node::NodeSubPub>("node_2", "/topic1", "/topic2"));
  node_list.emplace_back(std::make_shared<Node::NodeSubPub>("node_3", "/topic2", "/topic3"));
  node_list.emplace_back(std::make_shared<Node::NodeSub>("node_4", "/topic3"));

  for (auto & node : node_list) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}
