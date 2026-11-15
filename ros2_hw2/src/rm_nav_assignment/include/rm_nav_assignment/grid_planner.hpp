// -*- coding: utf-8 -*-
#pragma once
// 纯 C++ 的栅格路径规划核心，不依赖 ROS。
#ifndef RM_NAV_ASSIGNMENT__GRID_PLANNER_HPP_
#define RM_NAV_ASSIGNMENT__GRID_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace rm_nav_assignment {

struct Cell {
  int x = 0;
  int y = 0;
  bool operator==(const Cell & o) const { return x == o.x && y == o.y; }
  bool operator!=(const Cell & o) const { return !(*this == o); }
};

using Point = std::pair<double, double>;   // 世界坐标 (x, y)，单位米

class GridPlanner {
public:
  // 直接从 pgm + yaml 的参数加载地图
  bool loadFromPGM(const std::string & pgm_path, double resolution,
                   double origin_x, double origin_y, double occupied_thresh,
                   double free_thresh, bool negate, int safety_cells,
                   bool allow_unknown);

  // 从 nav_msgs::OccupancyGrid 的 data 加载（row 0 = 世界 y 最小处）
  void loadFromOccupancy(const std::vector<int8_t> & data, uint32_t width,
                         uint32_t height, double resolution, double origin_x,
                         double origin_y, int safety_cells, bool allow_unknown);

  // 学长强调的两个坐标转换函数
  Cell worldToGrid(double wx, double wy) const;
  Point gridToWorld(double gx, double gy) const;

  bool inBounds(int gx, int gy) const;
  bool blockedAt(int gx, int gy) const;
  Cell nearestFree(int gx, int gy, int radius = 40) const;
  bool hasFreeNear(int gx, int gy, int radius = 40) const;

  // A*：8 邻域 + octile 启发式，保证栅格意义下路径最短
  std::vector<Cell> astar(const Cell & start, const Cell & goal);

  // 世界坐标进、世界坐标出：直接得到 nav_msgs/Path 里要 push_back 的点
  std::vector<Point> plan(double start_x, double start_y,
                          double goal_x, double goal_y);

  // 原始三态地图（供发布 /map）与"膨胀过的代价地图"（供发布 /global_costmap/costmap）
  std::vector<int8_t> buildRawMap() const;
  std::vector<int8_t> buildCostmap() const;

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  size_t blockedCount() const;
  static double length(const std::vector<Point> & pts);

private:
  void classifyAndInflate(int safety_cells, bool allow_unknown);
  // 相邻两点间距太大时补点，让 RVIZ 连出来的线是连续的
  std::vector<Point> densify(const std::vector<Point> & pts, double step) const;

  uint32_t width_ = 0, height_ = 0;
  double resolution_ = 0.05, origin_x_ = 0.0, origin_y_ = 0.0;
  int safety_cells_ = 0;
  std::vector<uint8_t> occupied_, unknown_, blocked_, dist_;
};

}  // namespace rm_nav_assignment

#endif  // RM_NAV_ASSIGNMENT__GRID_PLANNER_HPP_
