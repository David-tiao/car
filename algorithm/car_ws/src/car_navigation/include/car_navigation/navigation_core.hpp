#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace car_navigation {

// ROS OccupancyGrid order: row 0 is the map's bottom row, x increases right.
struct Grid {
  int width = 0;
  int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  std::vector<std::int8_t> data; // -1 unknown, 0 free, 100 occupied
};

struct Cell { int x = 0; int y = 0; };
struct PathResult {
  bool success = false;
  std::string reason;
  std::vector<Cell> cells;
  double length_m = 0.0;
};

Grid load_trinary_pgm_yaml(const std::string& yaml_file);
bool matching_geometry(const Grid& a, const Grid& b);
bool world_to_cell(const Grid& map, double x, double y, Cell& cell);
std::pair<double, double> cell_center(const Grid& map, Cell cell);

// No diagonal corner cutting; unknown cells and masks are treated as blocked.
// clearance_m is a physical inflation radius, NOT a calibrated robot footprint.
PathResult plan_a_star(const Grid& physical, const Grid& keepout,
                       double start_x, double start_y,
                       double goal_x, double goal_y,
                       double clearance_m);
}  // namespace car_navigation
