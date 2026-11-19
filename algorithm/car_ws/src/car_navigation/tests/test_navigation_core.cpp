#include "car_navigation/navigation_core.hpp"
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
using namespace car_navigation;
int main() {
  Grid g;g.width=10;g.height=10;g.resolution=0.1;g.data.assign(100,0);
  Grid mask=g;
  for(int y=0;y<9;++y) mask.data[y*10+5]=100;
  auto path=plan_a_star(g,mask,0.15,0.15,0.85,0.15,0.0);
  assert(path.success && path.cells.size()>10 && path.length_m>1.2);
  assert(path.cells.front().x==1 && path.cells.back().x==8);
  auto no=mask;no.data[95]=100;
  assert(plan_a_star(g,no,0.15,0.15,0.85,0.15,0.0).reason=="NO_PATH");
  mask.data[11]=100;
  assert(plan_a_star(g,mask,0.15,0.15,0.85,0.15,0.0).reason=="START_BLOCKED_OR_TOO_CLOSE");
  assert(plan_a_star(g,mask,-0.1,0.15,0.85,0.15,0.0).reason=="START_OUT_OF_MAP");
  Grid another=mask;another.resolution=0.2;
  assert(plan_a_star(g,another,0.15,0.15,0.85,0.15,0.0).reason=="MAP_MASK_GEOMETRY_MISMATCH");
  Grid corner;corner.width=2;corner.height=2;corner.resolution=1.;corner.data={0,100,100,0};
  Grid cm=corner;cm.data={0,0,0,0};
  assert(plan_a_star(corner,cm,0.5,0.5,1.5,1.5,0).reason=="NO_PATH");
  Grid open;open.width=7;open.height=7;open.resolution=0.1;open.data.assign(49,0);
  Grid open_mask=open;open.data[3*7+3]=100;
  auto direct=plan_a_star(open,open_mask,0.15,0.35,0.55,0.35,0.0);
  auto inflated=plan_a_star(open,open_mask,0.15,0.35,0.55,0.35,0.11);
  assert(direct.success && inflated.success && inflated.length_m > direct.length_m);
  // Binary PGM bottom row white/free and top row black/occupied: reversal test.
  auto root=std::filesystem::temp_directory_path()/"car_navigation_core_test";
  std::filesystem::create_directories(root);
  {
    std::ofstream o(root/"m.pgm",std::ios::binary);
    o<<"P5\n2 2\n255\n";
    const unsigned char pixels[4]={0,0,255,255};
    o.write(reinterpret_cast<const char*>(pixels),4);
  }
  {
    std::ofstream o(root/"m.yaml");
    o<<"image: m.pgm\nmode: trinary\nresolution: 0.1\norigin: [-0.2, -0.2, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }
  auto img=load_trinary_pgm_yaml((root/"m.yaml").string());
  assert(img.width==2 && img.height==2 && img.data[0]==0 && img.data[3]==100);
  auto wc=cell_center(img,{1,0});
  assert(std::abs(wc.first+0.05)<1e-8 && std::abs(wc.second+0.15)<1e-8);
  std::filesystem::remove_all(root);
  std::cout<<"PASS navigation core: map PGM, coordinate conversion, A*, keepout, diagonal guard\n";
}
