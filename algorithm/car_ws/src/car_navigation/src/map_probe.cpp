// Pure CLI candidate route chooser; validates the CURRENT map + mask before publishing demo poses.
#include "car_navigation/navigation_core.hpp"
#include <array>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
int main(int argc,char**argv) {
  if(argc!=4) {
    std::cerr<<"Usage: car_map_probe map.yaml keepout.yaml clearance_m\n";return 2;
  }
  try {
    auto map=car_navigation::load_trinary_pgm_yaml(argv[1]);
    auto mask=car_navigation::load_trinary_pgm_yaml(argv[2]);
    const double c=std::stod(argv[3]);
    struct Candidate {const char* name;double sx,sy,gx,gy;};
    const std::array<Candidate,9> choices{{
      {"C3->C2",2.05,2.05,2.05,1.20},
      {"C2->C1",2.05,1.20,2.05,0.35},
      {"A3->A2",0.35,2.05,0.35,1.20},
      {"A2->A1",0.35,1.20,0.35,0.35},
      {"B3->B2",1.20,2.05,1.20,1.20},
      {"B2->B1",1.20,1.20,1.20,0.35},
      {"C2->B2",2.05,1.20,1.20,1.20},
      {"B2->A2",1.20,1.20,0.35,1.20},
      {"A1->B1",0.35,0.35,1.20,0.35}
    }};
    for(const auto&v:choices) {
      const auto result=car_navigation::plan_a_star(map,mask,v.sx,v.sy,v.gx,v.gy,c);
      if(result.success) {
        std::cerr<<"Selected validated sample route: "<<v.name<<" (static map only)\n";
        std::cout<<v.sx<<" "<<v.sy<<" "<<v.gx<<" "<<v.gy<<"\n";
        return 0;
      }
      std::cerr<<"Rejected candidate "<<v.name<<": "<<result.reason<<"\n";
    }
    std::cerr<<"ERROR: no candidate route passes map + keepout + clearance "<<c<<"m\n";
    return 3;
  } catch(const std::exception&e){std::cerr<<"ERROR: "<<e.what()<<"\n";return 4;}
}
