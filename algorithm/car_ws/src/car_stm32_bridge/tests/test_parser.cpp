#include "car_stm32_bridge/ulog43_parser.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

using namespace car_stm32_bridge;
static std::string sample() {
  std::array<double,43> ch{};
  ch[5]=123000;ch[16]=90;ch[28]=175;ch[29]=175;ch[30]=175;ch[31]=175;
  std::ostringstream out;
  for(std::size_t i=0;i<ch.size();++i){if(i)out<<',';out<<ch[i];}
  return out.str();
}
int main() {
  Ulog43Frame f;
  std::string err;
  assert(parse_ulog43(sample(),f,&err));
  assert(f.source_time_ms==123000);
  assert(f.ch[16]==90.0);
  auto b=wheels_to_body(f,1750.0,0.093,0.11);
  assert(std::fabs(b.vx_mps-0.1)<1e-8);
  assert(std::fabs(b.vy_mps)<1e-8);
  assert(std::fabs(b.wz_radps)<1e-8);
  assert(!parse_ulog43("1,2,3",f,&err));
  assert(!parse_ulog43(sample()+",8",f,&err));
  auto bad=sample();bad.replace(0,1,"n");
  assert(!parse_ulog43(bad,f,&err));
  std::cout << "ULog43 parser/kinematics: PASS\n";
}
