#include "car_stm32_bridge/ulog43_parser.hpp"
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace car_stm32_bridge {
namespace {
bool fail(std::string *error, const char *msg) {
  if(error) *error=msg;
  return false;
}
}
bool parse_ulog43(const std::string &line, Ulog43Frame &out, std::string *error) {
  if (line.empty() || line.size()>2048) return fail(error,"empty/oversized row");
  Ulog43Frame tmp;
  std::size_t begin=0;
  for (std::size_t i=0; i<43; ++i) {
    const std::size_t sep=line.find(',',begin);
    if (i==42 && sep!=std::string::npos) return fail(error,"more than 43 columns");
    if (i!=42 && sep==std::string::npos) return fail(error,"fewer than 43 columns");
    const std::string field=line.substr(begin,(sep==std::string::npos?line.size():sep)-begin);
    if(field.empty()) return fail(error,"missing value");
    char *end=nullptr;
    errno=0;
    const double v=std::strtod(field.c_str(),&end);
    if (errno==ERANGE || end==field.c_str() || !std::isfinite(v)) return fail(error,"invalid number");
    while (*end==' '||*end=='\t'||*end=='\r') ++end;
    if(*end!='\0') return fail(error,"invalid trailing field data");
    tmp.ch[i]=v;
    begin=sep==std::string::npos?line.size():sep+1;
  }
  const double ms=tmp.ch[5];
  if (ms<0 || ms>static_cast<double>(std::numeric_limits<std::uint32_t>::max()) ||
      std::floor(ms)!=ms) return fail(error,"invalid MCU timestamp");
  tmp.source_time_ms=static_cast<std::uint32_t>(ms);
  out=tmp;
  return true;
}
BodyVelocity wheels_to_body(const Ulog43Frame &f, double pulses_per_meter,
                            double wheelbase_half_m, double track_half_m) {
  BodyVelocity b;
  if (!(pulses_per_meter>0) || !(wheelbase_half_m>0) || !(track_half_m>0)) return b;
  const double fl=f.ch[28]/pulses_per_meter;
  const double fr=f.ch[29]/pulses_per_meter;
  const double rl=f.ch[30]/pulses_per_meter;
  const double rr=f.ch[31]/pulses_per_meter;
  const double L=wheelbase_half_m+track_half_m;
  b.vx_mps=(fl+fr+rl+rr)/4.0;
  b.vy_mps=(-fl+fr+rl-rr)/4.0;
  b.wz_radps=(-fl+fr-rl+rr)/(4.0*L);
  return b;
}
} // namespace car_stm32_bridge
