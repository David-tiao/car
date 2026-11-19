#include "car_serial_link/protocol.hpp"
#include <stdexcept>

namespace car_serial_link {
namespace {
std::uint16_t le16(const std::uint8_t *p) {
  return static_cast<std::uint16_t>(p[0]) | (static_cast<std::uint16_t>(p[1])<<8);
}
std::uint32_t le32(const std::uint8_t *p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1])<<8) |
         (static_cast<std::uint32_t>(p[2])<<16) | (static_cast<std::uint32_t>(p[3])<<24);
}
}
std::uint16_t crc16(const std::uint8_t *data,std::size_t n) {
  std::uint16_t crc=0xFFFF;
  for(std::size_t i=0;i<n;++i) {
    crc ^= static_cast<std::uint16_t>(data[i])<<8;
    for(int j=0;j<8;++j) crc=(crc&0x8000)?static_cast<std::uint16_t>((crc<<1)^0x1021):static_cast<std::uint16_t>(crc<<1);
  }
  return crc;
}
void Writer::u8(std::uint8_t x){data_.push_back(x);}
void Writer::u16(std::uint16_t x){u8(static_cast<std::uint8_t>(x));u8(static_cast<std::uint8_t>(x>>8));}
void Writer::i16(std::int16_t x){u16(static_cast<std::uint16_t>(x));}
void Writer::u32(std::uint32_t x){u16(static_cast<std::uint16_t>(x));u16(static_cast<std::uint16_t>(x>>16));}
void Writer::i32(std::int32_t x){u32(static_cast<std::uint32_t>(x));}
void Writer::text(const std::string &s){data_.insert(data_.end(),s.begin(),s.end());}
bool Reader::u8(std::uint8_t &v){if(remaining()<1)return false;v=bytes_[pos_++];return true;}
bool Reader::u16(std::uint16_t &v){if(remaining()<2)return false;v=le16(bytes_.data()+pos_);pos_+=2;return true;}
bool Reader::i16(std::int16_t &v){std::uint16_t raw;if(!u16(raw))return false;v=static_cast<std::int16_t>(raw);return true;}
bool Reader::u32(std::uint32_t &v){if(remaining()<4)return false;v=le32(bytes_.data()+pos_);pos_+=4;return true;}
bool Reader::i32(std::int32_t &v){std::uint32_t raw;if(!u32(raw))return false;v=static_cast<std::int32_t>(raw);return true;}
bool Reader::text(std::size_t len,std::string &s){if(remaining()<len)return false;s.assign(reinterpret_cast<const char*>(bytes_.data()+pos_),len);pos_+=len;return true;}
std::vector<std::uint8_t> encode(const Frame &f) {
  if(f.payload.size()>MAX_PAYLOAD)throw std::invalid_argument("CAR-LINK payload too large");
  Writer w;w.u8(0xA5);w.u8(0x5A);w.u8(VERSION);w.u8(f.type);w.u8(f.flags);
  w.u16(f.seq);w.u16(static_cast<std::uint16_t>(f.payload.size()));w.u32(f.timestamp_ms);
  std::vector<std::uint8_t> out=w.bytes();out.insert(out.end(),f.payload.begin(),f.payload.end());
  auto crc=crc16(out.data()+2,out.size()-2);out.push_back(static_cast<std::uint8_t>(crc));out.push_back(static_cast<std::uint8_t>(crc>>8));return out;
}
void Parser::reset(){bytes_.clear();}
void Parser::feed(const std::uint8_t *data,std::size_t n,std::vector<Frame>&out) {
  if(n==0)return;
  if(data==nullptr)throw std::invalid_argument("null buffer");
  bytes_.insert(bytes_.end(),data,data+n);
  if(bytes_.size()>8192){discarded_bytes_+=bytes_.size()-8192;bytes_.erase(bytes_.begin(),bytes_.end()-8192);}
  while(bytes_.size()>=2) {
    if(bytes_[0]!=0xA5 || bytes_[1]!=0x5A){bytes_.erase(bytes_.begin());++discarded_bytes_;continue;}
    if(bytes_.size()<13)break;
    if(bytes_[2]!=VERSION){++version_errors_;bytes_.erase(bytes_.begin());++discarded_bytes_;continue;}
    std::size_t len=le16(bytes_.data()+7);
    if(len>MAX_PAYLOAD){++length_errors_;bytes_.erase(bytes_.begin());++discarded_bytes_;continue;}
    std::size_t total=15+len;
    if(bytes_.size()<total)break;
    auto expected=crc16(bytes_.data()+2,11+len);
    auto actual=le16(bytes_.data()+13+len);
    if(expected!=actual){++crc_errors_;bytes_.erase(bytes_.begin());++discarded_bytes_;continue;}
    Frame f;f.type=bytes_[3];f.flags=bytes_[4];f.seq=le16(bytes_.data()+5);f.timestamp_ms=le32(bytes_.data()+9);
    f.payload.assign(bytes_.begin()+13,bytes_.begin()+13+len);
    out.push_back(std::move(f));++frames_ok_;bytes_.erase(bytes_.begin(),bytes_.begin()+total);
  }
}
}
