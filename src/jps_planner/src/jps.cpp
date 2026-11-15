#include "jps_planner/jps.hpp"

#include <cmath>
#include <deque>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace jps
{

JPS::JPS(const std::vector<int8_t> & grid, int width, int height, int occupied_thresh)
: grid_(grid), w_(width), h_(height), occ_thresh_(occupied_thresh)
{
}

bool JPS::walkable(int x, int y) const
{
  if (x < 0 || y < 0 || x >= w_ || y >= h_) {
    return false;
  }
  const int8_t v = grid_[static_cast<size_t>(y) * w_ + x];
  return v >= 0 && v < occ_thresh_;
}

double JPS::euclid(int x0, int y0, int x1, int y1)
{
  return std::hypot(static_cast<double>(x1 - x0), static_cast<double>(y1 - y0));
}

bool JPS::jump(int x, int y, int dx, int dy, const GridPos & goal, GridPos & out) const
{
  const int nx = x + dx;
  const int ny = y + dy;
  if (!walkable(nx, ny)) {
    return false;
  }
  if (nx == goal.x && ny == goal.y) {
    out = {nx, ny};
    return true;
  }

  if (dx != 0 && dy != 0) {
    // Diagonal move: forced neighbors adjacent to blocked cells.
    if ((!walkable(nx - dx, ny) && walkable(nx - dx, ny + dy)) ||
        (!walkable(nx, ny - dy) && walkable(nx + dx, ny - dy)))
    {
      out = {nx, ny};
      return true;
    }
    // If a jump point exists in either straight direction, this is one too.
    GridPos tmp;
    if (jump(nx, ny, dx, 0, goal, tmp) || jump(nx, ny, 0, dy, goal, tmp)) {
      out = {nx, ny};
      return true;
    }
  } else if (dx != 0) {
    // Horizontal move.
    if ((!walkable(nx, ny + 1) && walkable(nx + dx, ny + 1)) ||
        (!walkable(nx, ny - 1) && walkable(nx + dx, ny - 1)))
    {
      out = {nx, ny};
      return true;
    }
  } else {
    // Vertical move.
    if ((!walkable(nx + 1, ny) && walkable(nx + 1, ny + dy)) ||
        (!walkable(nx - 1, ny) && walkable(nx - 1, ny + dy)))
    {
      out = {nx, ny};
      return true;
    }
  }
  return jump(nx, ny, dx, dy, goal, out);
}

void JPS::identifySuccessors(
  const Node & node, const GridPos & goal, std::vector<GridPos> & out) const
{
  out.clear();
  if (node.parent == nullptr) {
    // Start node: expand all 8 walkable neighbors.
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        if (walkable(node.x + dx, node.y + dy)) {
          out.push_back({node.x + dx, node.y + dy});
        }
      }
    }
    return;
  }

  const int dx = sign(node.x - node.parent->x);
  const int dy = sign(node.y - node.parent->y);
  const int x = node.x;
  const int y = node.y;

  if (dx != 0 && dy != 0) {
    // Diagonal: 3 natural neighbors.
    if (walkable(x + dx, y)) {out.push_back({x + dx, y});}
    if (walkable(x, y + dy)) {out.push_back({x, y + dy});}
    if (walkable(x + dx, y + dy)) {out.push_back({x + dx, y + dy});}
    // 2 forced neighbors.
    if (!walkable(x - dx, y) && walkable(x - dx, y + dy)) {out.push_back({x - dx, y + dy});}
    if (!walkable(x, y - dy) && walkable(x + dx, y - dy)) {out.push_back({x + dx, y - dy});}
  } else if (dx == 0) {
    // Vertical: 1 natural neighbor + up to 2 forced.
    if (walkable(x, y + dy)) {out.push_back({x, y + dy});}
    if (!walkable(x + 1, y) && walkable(x + 1, y + dy)) {out.push_back({x + 1, y + dy});}
    if (!walkable(x - 1, y) && walkable(x - 1, y + dy)) {out.push_back({x - 1, y + dy});}
  } else {
    // Horizontal.
    if (walkable(x + dx, y)) {out.push_back({x + dx, y});}
    if (!walkable(x, y + 1) && walkable(x + dx, y + 1)) {out.push_back({x + dx, y + 1});}
    if (!walkable(x, y - 1) && walkable(x + dx, y - 1)) {out.push_back({x + dx, y - 1});}
  }
}

bool JPS::findPath(const GridPos & start, const GridPos & goal, std::vector<GridPos> & path)
{
  path.clear();
  expansions_ = 0;
  if (!walkable(start.x, start.y) || !walkable(goal.x, goal.y)) {
    return false;
  }

  struct OpenItem
  {
    double f;
    double h;
    Node * node;
  };
  struct OpenCmp
  {
    bool operator()(const OpenItem & a, const OpenItem & b) const
    {
      if (a.f != b.f) {return a.f > b.f;}
      return a.h > b.h;
    }
  };

  std::priority_queue<OpenItem, std::vector<OpenItem>, OpenCmp> open;
  std::unordered_map<int64_t, double> best_g;
  std::unordered_set<int64_t> closed;
  std::deque<Node> pool;

  auto key_of = [this](int x, int y) { return static_cast<int64_t>(y) * w_ + x; };

  pool.push_back(Node{start.x, start.y, 0.0, euclid(start.x, start.y, goal.x, goal.y), nullptr});
  open.push({pool.back().f, pool.back().f, &pool.back()});
  best_g[key_of(start.x, start.y)] = 0.0;

  Node * goal_node = nullptr;
  std::vector<GridPos> neighbors;

  while (!open.empty()) {
    Node * cur = open.top().node;
    open.pop();
    const int64_t ck = key_of(cur->x, cur->y);
    if (closed.count(ck)) {
      continue;
    }
    closed.insert(ck);
    ++expansions_;

    if (cur->x == goal.x && cur->y == goal.y) {
      goal_node = cur;
      break;
    }

    identifySuccessors(*cur, goal, neighbors);
    for (const GridPos & nb : neighbors) {
      const int dx = sign(nb.x - cur->x);
      const int dy = sign(nb.y - cur->y);
      GridPos jpt;
      if (!jump(cur->x, cur->y, dx, dy, goal, jpt)) {
        continue;
      }
      const double jg = cur->g + euclid(cur->x, cur->y, jpt.x, jpt.y);
      const int64_t jk = key_of(jpt.x, jpt.y);
      auto it = best_g.find(jk);
      if (it != best_g.end() && jg >= it->second) {
        continue;
      }
      best_g[jk] = jg;
      const double h = euclid(jpt.x, jpt.y, goal.x, goal.y);
      pool.push_back(Node{jpt.x, jpt.y, jg, jg + h, cur});
      open.push({jg + h, h, &pool.back()});
    }
  }

  if (goal_node == nullptr) {
    return false;
  }

  // Reconstruct: walk parent chain goal -> start, then reverse.
  std::vector<Node *> chain;
  for (Node * n = goal_node; n != nullptr; n = n->parent) {
    chain.push_back(n);
  }
  for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
    if (it == chain.rbegin()) {
      path.push_back({(*it)->x, (*it)->y});
      continue;
    }
    Node * prev = *(it - 1);
    Node * cur = *it;
    const int dx = sign(cur->x - prev->x);
    const int dy = sign(cur->y - prev->y);
    const int steps = std::max(std::abs(cur->x - prev->x), std::abs(cur->y - prev->y));
    for (int s = 1; s <= steps; ++s) {
      path.push_back({prev->x + dx * s, prev->y + dy * s});
    }
  }
  return true;
}

}  // namespace jps
