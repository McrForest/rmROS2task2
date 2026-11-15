#include <chrono>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "jps_planner/jps.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace jps_planner
{

struct MapMeta
{
  std::string image;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
};

class JpsPlannerNode : public rclcpp::Node
{
public:
  JpsPlannerNode()
  : Node("jps_planner_node")
  {
    declare_parameter<std::string>(
      "map_yaml", "/home/mcr/Workspace/RM/rosBasicsHomework2/mymap.yaml");
    declare_parameter<std::string>("map_frame", "map");
    // Robot footprint is robot_size x robot_size grid cells (square).
    // Obstacles are inflated by half of it so the point-robot planner
    // keeps enough clearance for the real footprint.
    declare_parameter<int>("robot_size", 10);
    frame_id_ = get_parameter("map_frame").as_string();
    const int robot_size = get_parameter("robot_size").as_int();
    inflation_radius_ = (robot_size + 1) / 2;  // 10 -> 5 cells

    if (!loadMap(get_parameter("map_yaml").as_string())) {
      throw std::runtime_error("Failed to load map.");
    }
    buildInflatedMap();

    // Latched (transient local) map so RViz can join at any time.
    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched);
    inflated_map_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("/inflated_map", latched);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", latched);
    marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/jps_markers", latched);

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&JpsPlannerNode::onInitialPose, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&JpsPlannerNode::onGoalPose, this, std::placeholders::_1));

    // Publish map once and keep republishing slowly so late subscribers get it.
    publishMaps();
    map_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&JpsPlannerNode::publishMaps, this));

    RCLCPP_INFO(
      get_logger(),
      "JPS planner ready. Map %dx%d (res %.3f), origin (%.3f, %.3f), "
      "robot %dx%d cells -> inflation radius %d. "
      "Use '2D Pose Estimate' for start and '2D Goal Pose' for goal in RViz.",
      width_, height_, resolution_, origin_x_, origin_y_,
      robot_size, robot_size, inflation_radius_);
  }

private:
  // ---------------- Map loading ----------------

  static std::string trim(const std::string & s)
  {
    const auto a = s.find_first_not_of(" \t\r\n");
    const auto b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
  }

  bool parseMapYaml(const std::string & path, MapMeta & meta)
  {
    std::ifstream in(path);
    if (!in) {
      RCLCPP_ERROR(get_logger(), "Cannot open map yaml: %s", path.c_str());
      return false;
    }
    std::string line;
    bool got_image = false, got_res = false, got_origin = false;
    while (std::getline(in, line) && !(got_image && got_res && got_origin)) {
      // Only parse top-level keys (no leading whitespace) to skip ROS param blocks.
      if (line.empty() || line[0] == ' ' || line[0] == '\t' || line[0] == '#') {
        continue;
      }
      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      const std::string key = trim(line.substr(0, colon));
      const std::string val = trim(line.substr(colon + 1));
      if (key == "image") {
        meta.image = val;
        got_image = true;
      } else if (key == "resolution") {
        meta.resolution = std::stod(val);
        got_res = true;
      } else if (key == "origin") {
        // format: [x, y, yaw]
        std::string v = val;
        for (auto & c : v) {if (c == '[' || c == ']' || c == ',') {c = ' ';}}
        std::istringstream ss(v);
        ss >> meta.origin_x >> meta.origin_y >> meta.origin_yaw;
        got_origin = true;
      } else if (key == "negate") {
        meta.negate = std::stoi(val);
      } else if (key == "occupied_thresh") {
        meta.occupied_thresh = std::stod(val);
      } else if (key == "free_thresh") {
        meta.free_thresh = std::stod(val);
      }
    }
    return got_image && got_res && got_origin;
  }

  // Read a P5 (binary) PGM into `pixels`.
  static bool readPgm(const std::string & path, int & w, int & h, std::vector<uint8_t> & pixels)
  {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
      return false;
    }
    // Token reader that skips whitespace and '#' comments.
    auto next_token = [&in]() -> std::string {
        std::string tok;
        char c;
        while (in.get(c)) {
          if (c == '#') {
            std::string dummy;
            std::getline(in, dummy);
            continue;
          }
          if (std::isspace(static_cast<unsigned char>(c))) {
            if (!tok.empty()) {
              break;
            }
            continue;
          }
          tok.push_back(c);
        }
        return tok;
      };
    if (next_token() != "P5") {
      return false;
    }
    w = std::stoi(next_token());
    h = std::stoi(next_token());
    const int maxval = std::stoi(next_token());
    if (maxval <= 0 || maxval > 255 || w <= 0 || h <= 0) {
      return false;
    }
    // Note: next_token() already consumed the single whitespace after maxval.
    pixels.resize(static_cast<size_t>(w) * h);
    in.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
    return in.gcount() == static_cast<std::streamsize>(pixels.size());
  }

  bool loadMap(const std::string & yaml_path)
  {
    MapMeta meta;
    if (!parseMapYaml(yaml_path, meta)) {
      return false;
    }
    resolution_ = meta.resolution;
    origin_x_ = meta.origin_x;
    origin_y_ = meta.origin_y;

    // Resolve image path relative to the yaml file directory.
    std::string img_path = meta.image;
    if (!img_path.empty() && img_path.front() != '/') {
      const auto slash = yaml_path.find_last_of('/');
      img_path =
        (slash == std::string::npos ? "" : yaml_path.substr(0, slash + 1)) + img_path;
    }

    std::vector<uint8_t> pixels;
    if (!readPgm(img_path, width_, height_, pixels)) {
      RCLCPP_ERROR(get_logger(), "Cannot read pgm image: %s", img_path.c_str());
      return false;
    }

    // Convert to ROS occupancy values, flipping rows (pgm is top-down,
    // OccupancyGrid data starts at the origin corner).
    occupancy_.assign(static_cast<size_t>(width_) * height_, -1);
    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        const uint8_t p = pixels[static_cast<size_t>(row) * width_ + col];
        const double occ = meta.negate ? p / 255.0 : (255 - p) / 255.0;
        int8_t v;
        if (occ > meta.occupied_thresh) {
          v = 100;
        } else if (occ < meta.free_thresh) {
          v = 0;
        } else {
          v = -1;  // unknown
        }
        occupancy_[static_cast<size_t>(height_ - 1 - row) * width_ + col] = v;
      }
    }
    return true;
  }

  // Build the planning grid: original obstacles plus an inflated ring of
  // `inflation_radius_` cells (Chebyshev distance, matching the 8-connected
  // motion model). Unknown cells count as obstacle sources too.
  // Display values: 100 = real obstacle, 70 = inflation zone, 0 = free.
  void buildInflatedMap()
  {
    inflated_.assign(occupancy_.size(), 0);
    std::deque<std::pair<int, int>> q;  // (x, y), ring grows by Chebyshev dist

    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int8_t v = occupancy_[static_cast<size_t>(y) * width_ + x];
        if (v < 0 || v >= 50) {
          inflated_[static_cast<size_t>(y) * width_ + x] = 100;
          q.emplace_back(x, y);
        }
      }
    }

    const size_t total_obstacles = q.size();
    // Level-order BFS: each ring marks its walkable 8-neighbors as inflated.
    for (int ring = 0; ring < inflation_radius_ && !q.empty(); ++ring) {
      const size_t ring_size = q.size();
      for (size_t i = 0; i < ring_size; ++i) {
        const auto [cx, cy] = q.front();
        q.pop_front();
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
              continue;
            }
            int8_t & cell = inflated_[static_cast<size_t>(ny) * width_ + nx];
            if (cell == 0) {
              cell = 70;  // inflation zone: blocked for planning
              q.emplace_back(nx, ny);
            }
          }
        }
      }
    }

    size_t blocked = 0;
    for (const int8_t v : inflated_) {
      if (v != 0) {++blocked;}
    }
    RCLCPP_INFO(
      get_logger(),
      "Inflated map built: %zu obstacle cells, %zu blocked after %d-ring "
      "inflation (Chebyshev).",
      total_obstacles, blocked, inflation_radius_);
  }

  void publishMaps()
  {
    auto make_msg = [&](const std::vector<int8_t> & data) {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = now();
        msg.header.frame_id = frame_id_;
        msg.info.resolution = resolution_;
        msg.info.width = width_;
        msg.info.height = height_;
        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.orientation.w = 1.0;
        msg.data = data;
        return msg;
      };
    map_pub_->publish(make_msg(occupancy_));
    inflated_map_pub_->publish(make_msg(inflated_));
  }

  // ---------------- Coordinate helpers ----------------

  bool worldToGrid(double wx, double wy, jps::GridPos & out) const
  {
    const int gx = static_cast<int>((wx - origin_x_) / resolution_);
    const int gy = static_cast<int>((wy - origin_y_) / resolution_);
    out = {gx, gy};
    return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
  }

  void gridToWorld(const jps::GridPos & g, double & wx, double & wy) const
  {
    wx = origin_x_ + (g.x + 0.5) * resolution_;
    wy = origin_y_ + (g.y + 0.5) * resolution_;
  }

  // Planning runs on the inflated grid: a cell is only usable if the
  // robot center can sit there with the full footprint clear of obstacles.
  bool gridFree(const jps::GridPos & g) const
  {
    if (g.x < 0 || g.y < 0 || g.x >= width_ || g.y >= height_) {
      return false;
    }
    return inflated_[static_cast<size_t>(g.y) * width_ + g.x] == 0;
  }

  // If the clicked cell is blocked, snap to the nearest free cell (spiral out).
  bool snapToFree(jps::GridPos & g, int max_radius = 25) const
  {
    if (gridFree(g)) {
      return true;
    }
    for (int r = 1; r <= max_radius; ++r) {
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != r) {
            continue;
          }
          jps::GridPos cand{g.x + dx, g.y + dy};
          if (gridFree(cand)) {
            g = cand;
            return true;
          }
        }
      }
    }
    return false;
  }

  // ---------------- RViz callbacks ----------------

  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_world_ = msg->pose.pose.position;
    has_start_ = true;
    RCLCPP_INFO(
      get_logger(), "Initial pose set: (%.3f, %.3f)",
      start_world_.x, start_world_.y);
    publishMarkers();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_start_) {
      RCLCPP_WARN(get_logger(), "Set the initial pose (2D Pose Estimate) first.");
      return;
    }
    goal_world_ = msg->pose.position;
    has_goal_ = true;
    RCLCPP_INFO(
      get_logger(), "Goal set: (%.3f, %.3f)", goal_world_.x, goal_world_.y);
    plan();
    publishMarkers();
  }

  // ---------------- Planning ----------------

  void plan()
  {
    jps::GridPos start, goal;
    if (!worldToGrid(start_world_.x, start_world_.y, start)) {
      RCLCPP_WARN(get_logger(), "Start pose is outside the map.");
      return;
    }
    if (!worldToGrid(goal_world_.x, goal_world_.y, goal)) {
      RCLCPP_WARN(get_logger(), "Goal pose is outside the map.");
      return;
    }
    if (!snapToFree(start)) {
      RCLCPP_WARN(get_logger(), "No free cell near the start pose.");
      return;
    }
    if (!snapToFree(goal)) {
      RCLCPP_WARN(get_logger(), "No free cell near the goal pose.");
      return;
    }

    // Threshold 1: everything but fully free (0) cells is blocked, i.e. the
    // inflation zone (70) and real obstacles (100) are untraversable.
    jps::JPS planner(inflated_, width_, height_, 1);
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<jps::GridPos> grid_path;
    const bool ok = planner.findPath(start, goal, grid_path);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (!ok) {
      RCLCPP_WARN(
        get_logger(), "JPS failed to find a path (%.2f ms, %d expansions).",
        ms, planner.expansions());
      return;
    }

    // Path length for the log.
    double length = 0.0;
    for (size_t i = 1; i < grid_path.size(); ++i) {
      length += std::hypot(
        static_cast<double>(grid_path[i].x - grid_path[i - 1].x),
        static_cast<double>(grid_path[i].y - grid_path[i - 1].y)) * resolution_;
    }
    RCLCPP_INFO(
      get_logger(),
      "JPS path found: %zu cells, %.2f m, %.2f ms (%d expansions).",
      grid_path.size(), length, ms, planner.expansions());

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = frame_id_;
    path_msg.poses.reserve(grid_path.size());
    for (const auto & g : grid_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      gridToWorld(g, ps.pose.position.x, ps.pose.position.y);
      ps.pose.orientation.w = 1.0;
      path_msg.poses.push_back(ps);
    }
    path_pub_->publish(path_msg);
  }

  // ---------------- Visualization ----------------

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;

    auto make_marker = [&](int id, const geometry_msgs::msg::Point & p,
                        float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_id_;
        m.header.stamp = now();
        m.ns = "jps";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position = p;
        m.pose.position.z = 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.18;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0f;
        return m;
      };

    if (has_start_) {
      arr.markers.push_back(make_marker(0, start_world_, 0.1f, 0.8f, 0.1f));
    }
    if (has_goal_) {
      arr.markers.push_back(make_marker(1, goal_world_, 0.9f, 0.15f, 0.1f));
    }
    if (!arr.markers.empty()) {
      marker_pub_->publish(arr);
    }
  }

  // ---------------- Members ----------------

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  std::string frame_id_ = "map";
  std::vector<int8_t> occupancy_;  // raw map from the pgm (for display)
  std::vector<int8_t> inflated_;   // planning grid: obstacles inflated
  int inflation_radius_ = 5;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  bool has_start_ = false;
  bool has_goal_ = false;
  geometry_msgs::msg::Point start_world_;
  geometry_msgs::msg::Point goal_world_;
};

}  // namespace jps_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<jps_planner::JpsPlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "jps_planner_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
