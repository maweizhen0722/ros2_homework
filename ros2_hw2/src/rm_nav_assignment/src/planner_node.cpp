// -*- coding: utf-8 -*-
// ROS2 节点（按 Humble 接口编写，已在虚拟机里编译启动过）：
//   订阅 /goal_pose（RVIZ 的 2D Goal Pose）-> 在 costmap 上跑 A* -> 发布 nav_msgs/Path
//   同时发布 /global_costmap/costmap（膨胀后的代价地图），RVIZ 里能看到"墙变粗了"
//
// 对应学长课上给的提示：
//   geometry_msgs::PoseStamped /goal_pose、nav_msgs::Path、push_back、
//   worldToGrid / gridToWorld 两组坐标转换、相邻点连成线。
#include "rm_nav_assignment/grid_planner.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using rm_nav_assignment::GridPlanner;
using std::placeholders::_1;

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : rclcpp::Node("rm_nav_planner") {
    // ---------------- 参数 ----------------
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/planned_path");
    costmap_topic_ = declare_parameter<std::string>("costmap_topic", "/global_costmap/costmap");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    start_x_ = declare_parameter<double>("start_x", 1.47);
    start_y_ = declare_parameter<double>("start_y", 0.97);
    safety_cells_ = declare_parameter<int>("safety_cells", 4);
    allow_unknown_ = declare_parameter<bool>("allow_unknown", false);
    map_file_ = declare_parameter<std::string>("map_file", "");
    map_resolution_ = declare_parameter<double>("map_resolution", 0.02);
    map_origin_x_ = declare_parameter<double>("map_origin_x", -8.54);
    map_origin_y_ = declare_parameter<double>("map_origin_y", -10.04);
    const double occ_thresh = declare_parameter<double>("map_occupied_thresh", 0.65);
    const double free_thresh = declare_parameter<double>("map_free_thresh", 0.196);

    // ---------------- 发布 / 订阅 ----------------
    auto latch = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, latch);
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, latch);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, latch);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, latch, std::bind(&PlannerNode::onMap, this, _1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&PlannerNode::onGoal, this, _1));
    init_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&PlannerNode::onInitialPose, this, _1));

    // ---------------- 不依赖 map_server：直接读 pgm ---------------
    if (!map_file_.empty()) {
      local_mode_ = true;
      if (!planner_.loadFromPGM(map_file_, map_resolution_, map_origin_x_, map_origin_y_,
                                occ_thresh, free_thresh, false, safety_cells_, allow_unknown_)) {
        RCLCPP_ERROR(get_logger(), "读取地图失败: %s", map_file_.c_str());
      } else {
        nav_msgs::msg::OccupancyGrid raw;
        fillInfo(raw);
        raw.data = planner_.buildRawMap();
        map_pub_->publish(raw);                 // 原始地图
        publishCostmap(raw);                    // 膨胀后的 costmap
        map_ready_ = true;
        RCLCPP_INFO(get_logger(),
          "已加载 %s : %u x %u 格, 分辨率 %.3f m/格, origin=(%.2f, %.2f)",
          map_file_.c_str(), planner_.width(), planner_.height(),
          planner_.resolution(), planner_.originX(), planner_.originY());
      }
    }
    RCLCPP_INFO(get_logger(),
      "规划节点就绪 | 起点(%.2f, %.2f) | 订阅 %s | 发布 %s 和 %s",
      start_x_, start_y_, goal_topic_.c_str(), path_topic_.c_str(),
      costmap_topic_.c_str());
  }

private:
  void fillInfo(nav_msgs::msg::OccupancyGrid & grid) {
    grid.header.stamp = now();
    grid.header.frame_id = frame_id_;
    grid.info.map_load_time = now();
    grid.info.resolution = static_cast<float>(planner_.resolution());
    grid.info.width = planner_.width();
    grid.info.height = planner_.height();
    grid.info.origin.position.x = planner_.originX();
    grid.info.origin.position.y = planner_.originY();
    grid.info.origin.orientation.w = 1.0;
  }

  void publishCostmap(const nav_msgs::msg::OccupancyGrid & base) {
    auto cm = base;
    cm.header.stamp = now();
    cm.data = planner_.buildCostmap();
    costmap_pub_->publish(cm);
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (local_mode_) return;                  // 本地模式已有地图，忽略自己的 /map
    if (msg->data.size() < msg->info.width * msg->info.height) {
      RCLCPP_WARN(get_logger(), "收到的地图数据不完整");
      return;
    }
    planner_.loadFromOccupancy(msg->data, msg->info.width, msg->info.height,
                               msg->info.resolution,
                               msg->info.origin.position.x,
                               msg->info.origin.position.y,
                               safety_cells_, allow_unknown_);
    publishCostmap(*msg);
    map_ready_ = true;
    RCLCPP_INFO(get_logger(), "已从 %s 加载 costmap: %u x %u 格",
                map_topic_.c_str(), msg->info.width, msg->info.height);
  }

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    start_x_ = msg->pose.pose.position.x;
    start_y_ = msg->pose.pose.position.y;
    RCLCPP_INFO(get_logger(), "起点更新为 (%.2f, %.2f) —— 来自 RVIZ 2D Pose Estimate",
                start_x_, start_y_);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "地图还没加载好，稍后再点目标点");
      return;
    }
    const double gx = msg->pose.position.x, gy = msg->pose.position.y;
    const auto t0 = std::chrono::steady_clock::now();
    const auto pts = planner_.plan(start_x_, start_y_, gx, gy);
    const auto t1 = std::chrono::steady_clock::now();
    if (pts.empty()) {
      RCLCPP_WARN(get_logger(), "目标 (%.2f, %.2f) 不可达，换个点试试", gx, gy);
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    path.poses.reserve(pts.size());
    for (const auto & p : pts) {
      geometry_msgs::msg::PoseStamped pose;   // Path 就是一串 PoseStamped
      pose.header = path.header;
      pose.pose.position.x = p.first;         // 必须是世界坐标(米)，不能是栅格坐标
      pose.pose.position.y = p.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(),
      "目标(%.2f, %.2f) -> 规划成功: %zu 个点, 长度 %.2f m, 耗时 %ld ms",
      gx, gy, pts.size(), GridPlanner::length(pts),
      static_cast<long>(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()));
  }

  GridPlanner planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_, map_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;

  std::string frame_id_, goal_topic_, path_topic_, costmap_topic_, map_topic_, map_file_;
  double start_x_ = 0.0, start_y_ = 0.0;
  double map_resolution_ = 0.02, map_origin_x_ = 0.0, map_origin_y_ = 0.0;
  int safety_cells_ = 4;
  bool allow_unknown_ = false, map_ready_ = false, local_mode_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
