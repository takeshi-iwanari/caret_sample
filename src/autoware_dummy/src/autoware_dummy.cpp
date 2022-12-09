#include <cstdlib>
#include <chrono>
#include <memory>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"


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
  NodePub(std::string node_name, std::string ns, std::string pub_topic_name, int period_ms = 10, int latency_ms = 1)
  : Node(node_name, ns), count_(0)
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
  NodeSub(std::string node_name, std::string ns, std::string sub_topic_name, int latency_ms = 2)
  : Node(node_name, ns)
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
  NodeSubPub(std::string node_name, std::string ns, std::string sub_topic_name, std::string pub_topic_name, int latency_ms = 2)
  : Node(node_name, ns)
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


class NodeSubSubPub : public rclcpp::Node
{
public:
  NodeSubSubPub(std::string node_name, std::string ns, std::string sub_topic_name_0, std::string sub_topic_name_1, std::string pub_topic_name, int latency_ms = 2)
  : Node(node_name, ns)
  {
    pub_ = create_publisher<std_msgs::msg::Int32>(pub_topic_name, QOS_HISTORY_SIZE);

    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_0 = std::bind(&NodeSubSubPub::topic_callback, this, std::placeholders::_1, 0);
    sub_0_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_0, QOS_HISTORY_SIZE, cb_0);
    std::function<void(const std_msgs::msg::Int32::UniquePtr)> cb_1 = std::bind(&NodeSubSubPub::topic_callback, this, std::placeholders::_1, 1);
    sub_1_ = create_subscription<std_msgs::msg::Int32>(sub_topic_name_1, QOS_HISTORY_SIZE, cb_1);
  }

private:
  void topic_callback(const std_msgs::msg::Int32::UniquePtr msg, int index)
  {
    (void)index;
    auto msg_pub = std_msgs::msg::Int32();
    msg_pub.data = msg->data;
    pub_->publish(msg_pub);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_0_, sub_1_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  std::vector<std::shared_ptr<rclcpp::Node>> node_list;
//sensing/lidar/top/
  node_list.emplace_back(std::make_shared<NodePub>(
    "velodyne_convert_node", "/sensing/lidar/top",
    "/sensing/lidar/left/pointcloud_raw_ex",
    100, 2));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "crop_box_filter_self", "/sensing/lidar/top",
    "/sensing/lidar/left/pointcloud_raw_ex",
    "/sensing/lidar/left/outlier_filtered/pointcloud"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "pointcloud", "/sensing/lidar/left/outlier_filtered",
    "/sensing/lidar/left/outlier_filtered/pointcloud",
    "/sensing/lidar/concatenated/pointcloud"));

  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "voxel_grid_downsample_filter", "/localization/util",
    "/sensing/lidar/concatenated/pointcloud",
    "/localization/pose_twist_fusion_filter/kinematic_state"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "stop_filter", "/localization/pose_twist_fusion_filter",
    "/localization/pose_twist_fusion_filter/kinematic_state",
    "/localization/kinematic_state"));

  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "crop_box_filter", "/perception/obstacle_segmentation",
    "/sensing/lidar/concatenated/pointcloud",
    "/perception/obstacle_segmentation/range_cropped/pointcloud"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "multi_object_tracker", "/perception/object_recognition/tracking",
    "/perception/obstacle_segmentation/range_cropped/pointcloud",
    "/perception/object_recognition/tracking/objects"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "map_based_prediction", "/perception/object_recognition/prediction",
    "/perception/object_recognition/tracking/objects",
    "/perception/object_recognition/objects"));

  node_list.emplace_back(std::make_shared<NodeSubSubPub>(
    "obstacle_avoidance_planner", "/planning/scenario_planning/lane_driving/motion_planning",
    "/perception/object_recognition/objects", "/localization/kinematic_state",
    "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "obstacle_stop_planner", "/planning/scenario_planning/lane_driving/motion_planning",
    "/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory",
    "/planning/scenario_planning/lane_driving/trajectory"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "motion_velocity_smoother", "/planning/scenario_planning",
    "/planning/scenario_planning/lane_driving/trajectory",
    "/planning/scenario_planning/trajectory"));

  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "controller_node_exe", "/control/trajectory_follower",
    "/planning/scenario_planning/trajectory",
    "/control/trajectory_follower/control_cmd"));
  node_list.emplace_back(std::make_shared<NodeSubPub>(
    "vehicle_cmd_gate", "/control",
    "/control/trajectory_follower/control_cmd",
    "/control/current_gate_mode"));
  node_list.emplace_back(std::make_shared<NodeSub>(
    "system_error_monitor", "/system",
    "/control/current_gate_mode"));

  for (auto & node : node_list) {
    executor->add_node(node);
  }

  executor->spin();
  rclcpp::shutdown();

  return 0;
}
