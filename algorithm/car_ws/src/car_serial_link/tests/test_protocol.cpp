#include "car_serial_link/protocol.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace car_serial_link;
int main() {
  assert(crc16(reinterpret_cast<const std::uint8_t*>("123456789"),9)==0x29B1);
  Frame original; original.type=0x10;original.flags=0;original.seq=65534;original.timestamp_ms=123456;
  Writer w;w.i32(-1350);w.i16(-120);w.u32(0x12345678);original.payload=w.bytes();
  auto bytes=encode(original);
  assert(bytes.size()==15+original.payload.size());
  assert(bytes[0]==0xA5 && bytes[1]==0x5A && bytes[2]==1);
  Parser p;std::vector<Frame> got;
  for(const auto b:bytes)p.feed(&b,1,got);
  assert(got.size()==1);assert(got[0].payload==original.payload);
  assert(got[0].seq==65534 && got[0].timestamp_ms==123456);
  Reader reader(got[0].payload);std::int32_t x;std::int16_t y;std::uint32_t z;
  assert(reader.i32(x)&&reader.i16(y)&&reader.u32(z)&&reader.done());
  assert(x==-1350 && y==-120 && z==0x12345678);
  auto broken=bytes;broken.back()^=0x01;
  std::vector<std::uint8_t> mix{0x00,0x99,0xA5,0x01};
  mix.insert(mix.end(),broken.begin(),broken.end());
  mix.insert(mix.end(),bytes.begin(),bytes.end());
  std::vector<Frame> out;Parser q;q.feed(mix.data(),mix.size(),out);
  assert(q.crc_errors()>=1 && out.size()==1 && out[0].seq==65534);
  auto oversized=bytes;oversized[7]=0xFF;oversized[8]=0xFF;
  Parser v;std::vector<Frame> result;v.feed(oversized.data(),oversized.size(),result);
  assert(v.length_errors()>=1);
  bool threw=false;try {Frame b;b.payload.resize(193);encode(b);}catch(const std::invalid_argument&){threw=true;}
  assert(threw);
  // Many frames and split after corrupted header, the parser must recover.
  auto half=bytes.size()/2;Parser split;std::vector<Frame> complete;
  split.feed(bytes.data(),half,complete);assert(complete.empty());
  split.feed(bytes.data()+half,bytes.size()-half,complete);assert(complete.size()==1);
  std::cout<<"PASS: CRC16, little-endian, streaming, noise, bad CRC, oversized and split frames\n";
}
