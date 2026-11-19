#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace car_serial_link {
constexpr std::uint8_t VERSION = 1;
constexpr std::size_t MAX_PAYLOAD = 192;
constexpr std::uint8_t FLAG_ACK_REQUEST = 0x01;

enum class Type : std::uint8_t {
  HELLO=0x01, HEARTBEAT=0x02, LINK_STATUS=0x03,
  CHASSIS=0x10, WHEELS=0x11, IMU=0x12, MECHANISM=0x13,
  POWER=0x14, VISION_MEASUREMENT=0x15, CHASSIS_TARGET=0x16,
  REQUEST_CONTROL=0x80, RELEASE_CONTROL=0x81, SET_VELOCITY=0x82,
  SET_POSITION=0x83, STOP=0x84, START_MISSION=0x85,
  CANCEL_MISSION=0x86, MECHANISM_COMMAND=0x87,
  COMMAND_ACK=0x90, COMMAND_REJECTED=0x91, COMMAND_STARTED=0x92,
  COMMAND_PROGRESS=0x93, COMMAND_COMPLETED=0x94, COMMAND_FAILED=0x95,
  CONTROL_STATUS=0x96, MISSION_EVENT=0x97, VISION_EVENT=0x98,
  FAULT_EVENT=0x99
};

struct Frame {
  std::uint8_t type=0;
  std::uint8_t flags=0;
  std::uint16_t seq=0;
  std::uint32_t timestamp_ms=0;
  std::vector<std::uint8_t> payload;
};

std::uint16_t crc16(const std::uint8_t *data, std::size_t size);
std::vector<std::uint8_t> encode(const Frame &frame);
class Parser {
 public:
  void feed(const std::uint8_t *data, std::size_t size, std::vector<Frame> &out);
  std::uint64_t frames_ok() const {return frames_ok_;}
  std::uint64_t crc_errors() const {return crc_errors_;}
  std::uint64_t length_errors() const {return length_errors_;}
  std::uint64_t version_errors() const {return version_errors_;}
  std::uint64_t discarded_bytes() const {return discarded_bytes_;}
  void reset();
 private:
  std::vector<std::uint8_t> bytes_;
  std::uint64_t frames_ok_=0, crc_errors_=0, length_errors_=0;
  std::uint64_t version_errors_=0, discarded_bytes_=0;
};

class Writer {
 public:
  void u8(std::uint8_t v);
  void u16(std::uint16_t v);
  void i16(std::int16_t v);
  void u32(std::uint32_t v);
  void i32(std::int32_t v);
  void text(const std::string &s);
  const std::vector<std::uint8_t>& bytes() const {return data_;}
 private: std::vector<std::uint8_t> data_;
};
class Reader {
 public:
  explicit Reader(const std::vector<std::uint8_t> &bytes):bytes_(bytes){}
  bool u8(std::uint8_t &v);
  bool u16(std::uint16_t &v);
  bool i16(std::int16_t &v);
  bool u32(std::uint32_t &v);
  bool i32(std::int32_t &v);
  bool text(std::size_t len,std::string &out);
  bool done() const {return pos_==bytes_.size();}
  std::size_t remaining() const {return bytes_.size()-pos_;}
 private: const std::vector<std::uint8_t> &bytes_; std::size_t pos_=0;
};
}
