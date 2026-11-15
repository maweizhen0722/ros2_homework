// -*- coding: utf-8 -*-
#include "rm_nav_assignment/grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <queue>

namespace rm_nav_assignment {
namespace {
constexpr double kSqrt2 = 1.4142135623730951;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Neighbor { int dy; int dx; double cost; };
const Neighbor kNeighbors[8] = {
  {0, 1, 1.0}, {-1, 1, kSqrt2}, {-1, 0, 1.0}, {-1, -1, kSqrt2},
  {0, -1, 1.0}, {1, -1, kSqrt2}, {1, 0, 1.0}, {1, 1, kSqrt2},
};

inline bool isSpace(uint8_t c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}
}  // namespace

// ------------------------- 地图加载 -------------------------
bool GridPlanner::loadFromPGM(const std::string & pgm_path, double resolution,
                              double origin_x, double origin_y,
                              double occupied_thresh, double free_thresh,
                              bool negate, int safety_cells, bool allow_unknown) {
  std::ifstream in(pgm_path, std::ios::binary);
  if (!in) return false;
  std::vector<uint8_t> buf((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
  size_t p = 0;
  auto skip = [&]() {
    while (p < buf.size()) {
      while (p < buf.size() && isSpace(buf[p])) ++p;
      if (p < buf.size() && buf[p] == '#') {
        while (p < buf.size() && buf[p] != '\n') ++p;
      } else {
        break;
      }
    }
  };
  auto read_num = [&]() -> int {
    skip();
    int v = 0;
    while (p < buf.size() && buf[p] >= '0' && buf[p] <= '9') {
      v = v * 10 + (buf[p] - '0');
      ++p;
    }
    return v;
  };
  skip();
  std::string magic;
  while (p < buf.size() && !isSpace(buf[p])) magic.push_back(char(buf[p++]));
  if (magic != "P5") return false;
  int w = read_num(), h = read_num(), maxval = read_num();
  if (w <= 0 || h <= 0 || maxval <= 0) return false;
  // PGM 头和数据之间恰好隔一个空白字符（兼容 CRLF）
  if (p < buf.size() && buf[p] == '\r') ++p;
  if (p < buf.size() && buf[p] == '\n') ++p;
  if (p + static_cast<size_t>(w) * h > buf.size()) return false;

  width_ = static_cast<uint32_t>(w);
  height_ = static_cast<uint32_t>(h);
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  const size_t n = static_cast<size_t>(w) * h;
  occupied_.assign(n, 0);
  unknown_.assign(n, 0);

  for (int row = 0; row < h; ++row) {
    for (int col = 0; col < w; ++col) {
      const int v = static_cast<int>(buf[p + row * w + col]) * 255 / maxval;
      const double occ_prob = negate ? v / 255.0 : 1.0 - v / 255.0;
      // PGM 的第 0 行在图像上方；OccupancyGrid 的第 0 行在世界 y 最小处
      const size_t idx = static_cast<size_t>(h - 1 - row) * w + col;
      if (occ_prob > occupied_thresh) occupied_[idx] = 1;
      else if (occ_prob >= free_thresh) unknown_[idx] = 1;   // mode: trinary
    }
  }
  classifyAndInflate(safety_cells, allow_unknown);
  return true;
}

void GridPlanner::loadFromOccupancy(const std::vector<int8_t> & data, uint32_t width,
                                    uint32_t height, double resolution, double origin_x,
                                    double origin_y, int safety_cells, bool allow_unknown) {
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  const size_t n = static_cast<size_t>(width) * height;
  occupied_.assign(n, 0);
  unknown_.assign(n, 0);
  for (size_t i = 0; i < n && i < data.size(); ++i) {
    if (data[i] < 0) { unknown_[i] = 1; continue; }
    const double occ_prob = data[i] / 100.0;
    if (occ_prob > 0.65) occupied_[i] = 1;
    else if (occ_prob >= 0.196) unknown_[i] = 1;
  }
  classifyAndInflate(safety_cells, allow_unknown);
}

void GridPlanner::classifyAndInflate(int safety_cells, bool allow_unknown) {
  const size_t n = static_cast<size_t>(width_) * height_;
  safety_cells_ = std::max(0, safety_cells);
  dist_.assign(n, static_cast<uint8_t>(std::min(255, safety_cells_ + 1)));
  blocked_.assign(n, 0);

  // 从所有障碍格做 8 邻域 BFS，得到到最近障碍的距离（切比雪夫距离）
  if (safety_cells_ > 0) {
    std::queue<size_t> q;
    for (size_t i = 0; i < n; ++i) {
      if (occupied_[i]) { dist_[i] = 0; q.push(i); }
    }
    const int w = static_cast<int>(width_), h = static_cast<int>(height_);
    while (!q.empty()) {
      const size_t idx = q.front(); q.pop();
      if (dist_[idx] >= safety_cells_) continue;
      const int r = static_cast<int>(idx / width_), c = static_cast<int>(idx % width_);
      for (const auto & nb : kNeighbors) {
        const int nr = r + nb.dy, nc = c + nb.dx;
        if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
        const size_t ni = static_cast<size_t>(nr) * width_ + nc;
        if (dist_[ni] > dist_[idx] + 1) {
          dist_[ni] = dist_[idx] + 1;
          q.push(ni);
        }
      }
    }
  }
  for (size_t i = 0; i < n; ++i) {
    blocked_[i] = (safety_cells_ > 0 && dist_[i] <= safety_cells_) ||
                  (occupied_[i] != 0) ||
                  (!allow_unknown && unknown_[i] != 0);
  }
}

std::vector<int8_t> GridPlanner::buildRawMap() const {
  const size_t n = static_cast<size_t>(width_) * height_;
  std::vector<int8_t> out(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (unknown_[i]) out[i] = -1;
    else if (occupied_[i]) out[i] = 100;
  }
  return out;
}

std::vector<int8_t> GridPlanner::buildCostmap() const {
  const size_t n = static_cast<size_t>(width_) * height_;
  std::vector<int8_t> out(n, 0);
  for (size_t i = 0; i < n; ++i) {
    if (unknown_[i]) { out[i] = -1; continue; }
    if (occupied_[i]) { out[i] = 100; continue; }
    if (safety_cells_ > 0 && dist_[i] <= safety_cells_) {
      // 越靠近墙代价越高，形成一圈渐变的安全缓冲带
      const double ratio = 1.0 - dist_[i] / static_cast<double>(safety_cells_ + 1);
      out[i] = static_cast<int8_t>(20 + 60 * ratio);
    }
  }
  return out;
}

size_t GridPlanner::blockedCount() const {
  return std::count(blocked_.begin(), blocked_.end(), uint8_t{1});
}

// ------------------------- 坐标转换（学长重点强调） -------------------------
Cell GridPlanner::worldToGrid(double wx, double wy) const {
  return Cell{static_cast<int>(std::floor((wx - origin_x_) / resolution_)),
              static_cast<int>(std::floor((wy - origin_y_) / resolution_))};
}

Point GridPlanner::gridToWorld(double gx, double gy) const {
  return Point{origin_x_ + (gx + 0.5) * resolution_,
               origin_y_ + (gy + 0.5) * resolution_};
}

bool GridPlanner::inBounds(int gx, int gy) const {
  return gx >= 0 && gy >= 0 &&
         gx < static_cast<int>(width_) && gy < static_cast<int>(height_);
}

bool GridPlanner::blockedAt(int gx, int gy) const {
  if (!inBounds(gx, gy)) return true;
  return blocked_[static_cast<size_t>(gy) * width_ + gx] != 0;
}

bool GridPlanner::hasFreeNear(int gx, int gy, int radius) const {
  if (inBounds(gx, gy) && !blockedAt(gx, gy)) return true;
  for (int r = 1; r <= radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
        if (inBounds(gx + dx, gy + dy) && !blockedAt(gx + dx, gy + dy)) return true;
      }
    }
  }
  return false;
}

Cell GridPlanner::nearestFree(int gx, int gy, int radius) const {
  if (inBounds(gx, gy) && !blockedAt(gx, gy)) return Cell{gx, gy};
  for (int r = 1; r <= radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
        if (inBounds(gx + dx, gy + dy) && !blockedAt(gx + dx, gy + dy))
          return Cell{gx + dx, gy + dy};
      }
    }
  }
  return Cell{-1, -1};
}

// ------------------------- A* -------------------------
std::vector<Cell> GridPlanner::astar(const Cell & start, const Cell & goal) {
  const size_t n = static_cast<size_t>(width_) * height_;
  const int w = static_cast<int>(width_), h = static_cast<int>(height_);
  if (n == 0 || blockedAt(start.x, start.y) || blockedAt(goal.x, goal.y))
    return {};
  const size_t si = static_cast<size_t>(start.y) * width_ + start.x;
  const size_t gi = static_cast<size_t>(goal.y) * width_ + goal.x;

  auto heuristic = [&](int r, int c) {
    const double dx = std::abs(c - goal.x), dy = std::abs(r - goal.y);
    return dx + dy + (kSqrt2 - 2.0) * std::min(dx, dy);
  };
  struct QNode {
    double f, g;
    size_t idx;
    bool operator>(const QNode & o) const { return f > o.f; }
  };
  std::vector<double> g_score(n, kInf);
  std::vector<int32_t> parent(n, -1);
  std::vector<uint8_t> closed(n, 0);
  std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> heap;
  g_score[si] = 0.0;
  heap.push(QNode{heuristic(start.y, start.x), 0.0, si});

  while (!heap.empty()) {
    const QNode cur = heap.top(); heap.pop();
    if (closed[cur.idx] || cur.g > g_score[cur.idx] + 1e-9) continue;
    if (cur.idx == gi) break;
    closed[cur.idx] = 1;
    const int r = static_cast<int>(cur.idx / width_), c = static_cast<int>(cur.idx % width_);
    for (const auto & nb : kNeighbors) {
      const int nr = r + nb.dy, nc = c + nb.dx;
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const size_t ni = static_cast<size_t>(nr) * width_ + nc;
      if (closed[ni] || blocked_[ni]) continue;
      // 禁止贴着墙角斜穿
      if (nb.dy != 0 && nb.dx != 0 &&
          (blocked_[static_cast<size_t>(r) * width_ + nc] ||
           blocked_[static_cast<size_t>(nr) * width_ + c])) continue;
      const double ng = cur.g + nb.cost;
      if (ng < g_score[ni] - 1e-9) {
        g_score[ni] = ng;
        parent[ni] = static_cast<int32_t>(cur.idx);
        heap.push(QNode{ng + heuristic(nr, nc), ng, ni});
      }
    }
  }
  if (gi != si && !std::isfinite(g_score[gi])) return {};
  std::vector<Cell> path;
  int32_t idx = static_cast<int32_t>(gi);
  while (idx != -1) {
    path.push_back(Cell{static_cast<int>(idx % width_), static_cast<int>(idx / width_)});
    if (idx == static_cast<int32_t>(si)) break;
    idx = parent[idx];
  }
  std::reverse(path.begin(), path.end());
  if (path.empty() || path.front() != start) return {};
  return path;
}


std::vector<Point> GridPlanner::densify(const std::vector<Point> & pts,
                                        double step) const {
  std::vector<Point> out;
  if (pts.size() < 2) return pts;
  out.push_back(pts.front());
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const Point & p = pts[i];
    const Point & q = pts[i + 1];
    const double dist = std::hypot(q.first - p.first, q.second - p.second);
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int k = 1; k <= n; ++k) {
      out.emplace_back(p.first + (q.first - p.first) * k / n,
                       p.second + (q.second - p.second) * k / n);
    }
  }
  return out;
}

// ------------------------- 一步到位的规划接口 -------------------------
std::vector<Point> GridPlanner::plan(double start_x, double start_y,
                                     double goal_x, double goal_y) {
  const Cell s0 = worldToGrid(start_x, start_y);
  const Cell g0 = worldToGrid(goal_x, goal_y);
  if (!hasFreeNear(s0.x, s0.y) || !hasFreeNear(g0.x, g0.y)) return {};
  const Cell s = nearestFree(s0.x, s0.y);
  const Cell g = nearestFree(g0.x, g0.y);
  const auto raw = astar(s, g);
  if (raw.empty()) return {};
  std::vector<Point> raw_pts;
  raw_pts.reserve(raw.size());
  for (const auto & c : raw) raw_pts.emplace_back(c.x, c.y);
  const auto grid_pts = densify(raw_pts, 0.5);
  std::vector<Point> world;
  world.reserve(grid_pts.size());
  for (const auto & p : grid_pts) world.push_back(gridToWorld(p.first, p.second));
  return world;
}

double GridPlanner::length(const std::vector<Point> & pts) {
  double s = 0.0;
  for (size_t i = 1; i < pts.size(); ++i)
    s += std::hypot(pts[i].first - pts[i - 1].first,
                    pts[i].second - pts[i - 1].second);
  return s;
}

}  // namespace rm_nav_assignment
