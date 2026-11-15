#ifndef JPS_PLANNER__JPS_HPP_
#define JPS_PLANNER__JPS_HPP_

#include <cstdint>
#include <vector>

namespace jps
{

struct GridPos
{
  int x;
  int y;
};

// Jump Point Search on an 8-connected occupancy grid.
// Grid convention: row-major, index = y * width + x.
// Values follow ROS OccupancyGrid: -1 unknown, 0..100 probability.
class JPS
{
public:
  // Cells with value >= occupied_thresh, or unknown (-1), are blocked.
  explicit JPS(
    const std::vector<int8_t> & grid, int width, int height,
    int occupied_thresh = 50);

  // Plan from start to goal. On success returns true and fills `path`
  // with grid cells from start to goal (inclusive), with straight line
  // segments between jump points already interpolated.
  bool findPath(const GridPos & start, const GridPos & goal, std::vector<GridPos> & path);

  // Number of jump-point expansions performed by the last findPath call.
  int expansions() const { return expansions_; }

private:
  struct Node
  {
    int x;
    int y;
    double g;
    double f;
    Node * parent;
  };

  bool walkable(int x, int y) const;
  static int sign(int v) { return (v > 0) - (v < 0); }
  static double euclid(int x0, int y0, int x1, int y1);

  // Recursively jump from (x,y) along (dx,dy) until a jump point is found.
  bool jump(int x, int y, int dx, int dy, const GridPos & goal, GridPos & out) const;

  // Pruned neighbor generation according to the parent's move direction.
  void identifySuccessors(
    const Node & node, const GridPos & goal, std::vector<GridPos> & out) const;

  const std::vector<int8_t> & grid_;
  int w_;
  int h_;
  int occ_thresh_;
  int expansions_ = 0;
};

}  // namespace jps

#endif  // JPS_PLANNER__JPS_HPP_
