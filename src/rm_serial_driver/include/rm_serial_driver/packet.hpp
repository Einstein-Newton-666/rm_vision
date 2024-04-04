// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0xBE;
  float pitch;
  float yaw;
  float roll;
  float shoot_speed;
  uint8_t current_color;
  uint8_t enemies_blood_0;
  uint8_t enemies_blood_1;
  uint8_t enemies_blood_2;
  uint8_t enemies_blood_3;
  uint8_t enemies_blood_4;
  uint8_t enemies_blood_5;
  uint8_t enemies_outpost;
  uint8_t ifattackengineer;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xBE;  
  float pitch;
  float yaw;
  float pitch_speed;
  float yaw_speed;
  uint16_t checksum = 0;
  uint8_t tail = 0xED;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
