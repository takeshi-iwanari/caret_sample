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

  node_list.emplace_back(std::make_shared<Node::NodePub>("node_src", "/topic_src", 10, 0));
  node_list.emplace_back(std::make_shared<Node::NodeSubPub>("node_src0", "/topic_src", "/topic_src_0", 2));
  node_list.emplace_back(std::make_shared<Node::NodeSubPub>("node_src1", "/topic_src", "/topic_src_1", 4));
  node_list.emplace_back(std::make_shared<Node::NodeSubPub>("node_src2", "/topic_src", "/topic_src_2", 6));

  node_list.emplace_back(std::make_shared<Node::NodeSub3Pub>("node_sub3pub", "/topic_src_0", "/topic_src_1", "/topic_src_2", "/topic_sub3pub"));
  
  node_list.emplace_back(std::make_shared<Node::NodeSub>("node_dst", "/topic_sub3pub"));

  for (auto & node : node_list) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}
