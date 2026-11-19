#include "car_navigation/navigation_core.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace car_navigation {
namespace {
std::string strip(std::string v) {
  auto a = v.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return {};
  auto b = v.find_last_not_of(" \t\r\n");
  v = v.substr(a, b - a + 1);
  if (v.size() >= 2 && ((v.front() == '\'' && v.back() == '\'') ||
                         (v.front() == '"' && v.back() == '"'))) v = v.substr(1, v.size() - 2);
  return v;
}
std::string next_pgm_token(std::istream& in) {
  std::string s;
  while (in >> s) {
    if (s[0] == '#') { std::string rest; std::getline(in, rest); continue; }
    return s;
  }
  throw std::runtime_error("incomplete PGM header or pixels");
}
struct Pgm { int width=0, height=0; std::vector<std::uint8_t> data; };
Pgm read_pgm(const std::filesystem::path& p) {
  std::ifstream f(p, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open image: " + p.string());
  const auto magic = next_pgm_token(f);
  if (magic != "P5" && magic != "P2") throw std::runtime_error("PGM must be P5 or P2");
  const long w = std::stol(next_pgm_token(f)), h = std::stol(next_pgm_token(f));
  const long maxv = std::stol(next_pgm_token(f));
  if (w < 1 || h < 1 || w > 5000 || h > 5000 || maxv != 255 || w * h > 25000000L)
    throw std::runtime_error("unsupported PGM dimensions or maxval; expected 8-bit 255");
  Pgm out;
  out.width = static_cast<int>(w); out.height = static_cast<int>(h);
  out.data.resize(static_cast<std::size_t>(w*h));
  if (magic == "P5") {
    // The token extraction left the required one-byte separator unread.
    const int separator = f.get();
    if (separator == EOF || !std::isspace(static_cast<unsigned char>(separator)))
      throw std::runtime_error("missing P5 pixel separator");
    f.read(reinterpret_cast<char*>(out.data.data()), static_cast<std::streamsize>(out.data.size()));
    if (f.gcount() != static_cast<std::streamsize>(out.data.size()))
      throw std::runtime_error("truncated P5 PGM image");
  } else {
    for (auto& v : out.data) {
      long raw = std::stol(next_pgm_token(f));
      if (raw < 0 || raw > 255) throw std::runtime_error("PGM pixel outside 0..255");
      v = static_cast<std::uint8_t>(raw);
    }
  }
  return out;
}
std::vector<double> parse_origin(std::string s) {
  s = strip(s);
  if (s.size() < 5 || s.front() != '[' || s.back() != ']')
    throw std::runtime_error("invalid map origin [x,y,yaw]");
  s = s.substr(1, s.size() - 2);
  std::replace(s.begin(), s.end(), ',', ' ');
  std::istringstream st(s);
  std::vector<double> v;
  double n;
  while (st >> n) v.push_back(n);
  if (v.size() != 3) throw std::runtime_error("map origin needs 3 numbers");
  return v;
}
}  // namespace

Grid load_trinary_pgm_yaml(const std::string& yaml_file) {
  std::ifstream in(yaml_file);
  if (!in) throw std::runtime_error("cannot open map YAML: " + yaml_file);
  std::string image, mode, line;
  double resolution=0.0, occ=0.65, free=0.196;
  int negate=0;
  std::vector<double> origin;
  bool has_resolution=false, has_origin=false, has_image=false;
  while (std::getline(in, line)) {
    if (auto p = line.find('#'); p != std::string::npos) line.erase(p);
    auto p = line.find(':');
    if (p == std::string::npos) continue;
    const auto key=strip(line.substr(0,p)), val=strip(line.substr(p+1));
    if (key == "image") { image=val; has_image=true; }
    else if (key == "mode") mode=val;
    else if (key == "resolution") { resolution=std::stod(val); has_resolution=true; }
    else if (key == "origin") { origin=parse_origin(val); has_origin=true; }
    else if (key == "negate") negate=std::stoi(val);
    else if (key == "occupied_thresh") occ=std::stod(val);
    else if (key == "free_thresh") free=std::stod(val);
  }
  if (!has_image || !has_resolution || !has_origin || mode != "trinary" ||
      !std::isfinite(resolution) || resolution <= 0 || !std::isfinite(occ) ||
      !std::isfinite(free) || !(0 <= free && free < occ && occ <= 1) ||
      negate < 0 || negate > 1 || !std::all_of(origin.begin(), origin.end(),
                                              [](double x){return std::isfinite(x);}))
    throw std::runtime_error("invalid / unsupported trinary map YAML");
  if (std::abs(origin[2]) > 1e-7)
    throw std::runtime_error("rotated map origin not supported in preview planner");
  const auto path = std::filesystem::path(yaml_file).parent_path() / image;
  const Pgm img = read_pgm(path);
  Grid g;
  g.width=img.width; g.height=img.height;
  g.resolution=resolution; g.origin_x=origin[0]; g.origin_y=origin[1];
  g.origin_yaw=origin[2];
  g.data.assign(img.data.size(), -1);
  for (int y=0; y<g.height; ++y) {
    for (int x=0; x<g.width; ++x) {
      // PGM origin is upper-left; OccupancyGrid row zero is bottom-left.
      const auto pix=img.data[static_cast<std::size_t>((g.height-1-y)*g.width+x)];
      const double occupied=negate ? pix/255.0 : (255.0-pix)/255.0;
      std::int8_t value=-1;
      if (occupied > occ) value=100;
      else if (occupied < free) value=0;
      g.data[static_cast<std::size_t>(y*g.width+x)]=value;
    }
  }
  return g;
}

bool matching_geometry(const Grid& a, const Grid& b) {
  return a.width==b.width && a.height==b.height &&
         std::abs(a.resolution-b.resolution)<1e-9 &&
         std::abs(a.origin_x-b.origin_x)<1e-7 &&
         std::abs(a.origin_y-b.origin_y)<1e-7 &&
         std::abs(a.origin_yaw-b.origin_yaw)<1e-7 &&
         a.data.size()==b.data.size();
}

bool world_to_cell(const Grid& g, double x, double y, Cell& cell) {
  if (!std::isfinite(x) || !std::isfinite(y) || g.resolution <= 0) return false;
  const double col=(x-g.origin_x)/g.resolution, row=(y-g.origin_y)/g.resolution;
  if(col<0 || row<0 || col>=g.width || row>=g.height) return false;
  cell={static_cast<int>(std::floor(col)),static_cast<int>(std::floor(row))}; return true;
}
std::pair<double,double> cell_center(const Grid& g, Cell c) {
  return {g.origin_x+(c.x+0.5)*g.resolution, g.origin_y+(c.y+0.5)*g.resolution};
}

PathResult plan_a_star(const Grid& physical, const Grid& keepout,
                       double sx, double sy, double gx, double gy,
                       double clearance_m) {
  PathResult result;
  if (!matching_geometry(physical,keepout) || physical.width<1 || physical.height<1 ||
      physical.resolution<=0 || physical.data.size()!=
        static_cast<std::size_t>(physical.width)*static_cast<std::size_t>(physical.height)) {
    result.reason="MAP_MASK_GEOMETRY_MISMATCH"; return result;
  }
  if (!std::isfinite(clearance_m) || clearance_m < 0 || clearance_m > 0.5 ||
      clearance_m/physical.resolution>64.0) {
    result.reason="INVALID_CLEARANCE"; return result;
  }
  Cell start, goal;
  if (!world_to_cell(physical,sx,sy,start)) {result.reason="START_OUT_OF_MAP";return result;}
  if (!world_to_cell(physical,gx,gy,goal)) {result.reason="GOAL_OUT_OF_MAP";return result;}
  const int w=physical.width,h=physical.height;
  const std::size_t size=physical.data.size();
  std::vector<std::uint8_t> blocked(size,0);
  for (std::size_t i=0;i<size;++i) {
    const int p=physical.data[i], k=keepout.data[i];
    blocked[i] = (p<0 || p>=65 || k!=0) ? 1 : 0;
  }
  if (clearance_m > 0) {
    const int r=static_cast<int>(std::ceil(clearance_m/physical.resolution));
    std::vector<std::uint8_t> inflated=blocked;
    // For every obstacle, dilate by physical radius; kept simple for 280x280 grids.
    std::vector<std::pair<int,int>> shifts;
    for (int dy=-r;dy<=r;++dy) for(int dx=-r;dx<=r;++dx)
      if (static_cast<double>(dx*dx+dy*dy)*physical.resolution*physical.resolution <=
          clearance_m*clearance_m + 1e-12) shifts.emplace_back(dx,dy);
    for (int y=0;y<h;++y) for(int x=0;x<w;++x) {
      if (!blocked[static_cast<std::size_t>(y*w+x)]) continue;
      for (auto d:shifts) {
        const int xx=x+d.first, yy=y+d.second;
        if (xx>=0 && yy>=0 && xx<w && yy<h)
          inflated[static_cast<std::size_t>(yy*w+xx)]=1;
      }
    }
    blocked.swap(inflated);
  }
  const auto index=[w](Cell c){return c.y*w+c.x;};
  const int source=index(start), target=index(goal);
  if (blocked[source]) {result.reason="START_BLOCKED_OR_TOO_CLOSE";return result;}
  if (blocked[target]) {result.reason="GOAL_BLOCKED_OR_TOO_CLOSE";return result;}
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pq;
  std::vector<double> distance(size,std::numeric_limits<double>::infinity());
  std::vector<int> parent(size,-1);
  std::vector<std::uint8_t> visited(size,0);
  auto heuristic=[&](int i){
    int x=i%w,y=i/w;
    int dx=std::abs(x-goal.x),dy=std::abs(y-goal.y);
    return static_cast<double>(std::max(dx,dy))+(std::sqrt(2.0)-1.0)*std::min(dx,dy);
  };
  distance[source]=0;pq.push({heuristic(source),source});
  constexpr int step_x[8]={1,-1,0,0,1,1,-1,-1};
  constexpr int step_y[8]={0,0,1,-1,1,-1,1,-1};
  while (!pq.empty()) {
    const int at=pq.top().second;pq.pop();
    if (visited[at]) continue;
    visited[at]=1;
    if (at==target) break;
    const int x=at%w,y=at/w;
    for (int d=0;d<8;++d) {
      int nx=x+step_x[d], ny=y+step_y[d];
      if(nx<0||ny<0||nx>=w||ny>=h)continue;
      int ni=ny*w+nx;
      if(blocked[ni]||visited[ni])continue;
      if(step_x[d]!=0&&step_y[d]!=0 &&
          (blocked[static_cast<std::size_t>(y*w+nx)]||
           blocked[static_cast<std::size_t>(ny*w+x)])) continue;
      const double next=distance[at]+(d<4?1.0:std::sqrt(2.0));
      if(next+1e-10<distance[ni]) {
        distance[ni]=next;parent[ni]=at;
        pq.push({next+heuristic(ni),ni});
      }
    }
  }
  if(!visited[target]){result.reason="NO_PATH";return result;}
  for(int at=target;at!=-1;at=parent[at]) {
    result.cells.push_back({at%w,at/w});
    if(at==source)break;
  }
  std::reverse(result.cells.begin(),result.cells.end());
  result.success=true;result.reason="OK";
  result.length_m=distance[target]*physical.resolution;
  return result;
}
}  // namespace car_navigation
