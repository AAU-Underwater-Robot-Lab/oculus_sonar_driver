// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#pragma once

#include <vector>

#include "liboculus/Constants.h"
#include "liboculus/SimplePingResult.h"
// Only keep marine_acoustic_msgs
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"

#include "rclcpp/rclcpp.hpp"

namespace oculus_sonar_driver {

// Packs an acoustic_msgs::ProjectedSonarImage from the contents of a
// SimplePingResult
//
// Calling function is expected to fill in header, as required
// data is not found in the SimplePingResult.
//
// \todo Currently has no way to indicate failure...
template <typename PingT>
marine_acoustic_msgs::msg::ProjectedSonarImage pingToSonarImage(
    const liboculus::SimplePingResult<PingT> &ping) {
  marine_acoustic_msgs::msg::ProjectedSonarImage sonar_image;

  sonar_image.ping_info.frequency = ping.ping()->frequency;
  // QUESTION(lindzey): Is there a way to find out what sound speed
  //    the Oculus used when computing ranges? I don't like leaving
  //    this as the default value.
  // sonar_image.ping_info.sound_speed = ????
  const int num_bearings = ping.ping()->nBeams;
  const int num_ranges = ping.ping()->nRanges;

  // These fields are frequency dependent
  if ((sonar_image.ping_info.frequency > 2900000) &&
      (sonar_image.ping_info.frequency < 3100000)) {
    // 3.0 MHz
    sonar_image.ping_info.rx_beamwidths = std::vector<float>(
        num_bearings, liboculus::Oculus_3000MHz::AzimuthBeamwidthRad);
    sonar_image.ping_info.tx_beamwidths = std::vector<float>(
        num_bearings, liboculus::Oculus_3000MHz::ElevationBeamwidthRad);
  } else if ((sonar_image.ping_info.frequency > 2000000) &&
             (sonar_image.ping_info.frequency < 2200000)) {
    // 2.1 MHz
    sonar_image.ping_info.rx_beamwidths = std::vector<float>(
        num_bearings, liboculus::Oculus_2100MHz::AzimuthBeamwidthRad);
    sonar_image.ping_info.tx_beamwidths = std::vector<float>(
        num_bearings, liboculus::Oculus_2100MHz::ElevationBeamwidthRad);
  } else if ((sonar_image.ping_info.frequency > 1100000) &&
             (sonar_image.ping_info.frequency < 1300000)) {
    // 1.2 MHz
    sonar_image.ping_info.rx_beamwidths = std::vector<float>(
        num_bearings, liboculus::Oculus_1200MHz::AzimuthBeamwidthRad);
    sonar_image.ping_info.tx_beamwidths = std::vector<float>(
        num_bearings, liboculus::Oculus_1200MHz::ElevationBeamwidthRad);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ping_to_sonar_image"), "Unsupported frequency received from oculus: %d. Not publishing ProjectedSonarImage", sonar_image.ping_info.frequency);
  }

  sonar_image.beam_directions.resize(num_bearings);
  for (unsigned int idx = 0; idx < num_bearings; idx++) {
    float az = ping.bearings().at_rad(idx);
    sonar_image.beam_directions[idx].x = 0.0;  // Assuming elevation is 0
    sonar_image.beam_directions[idx].y = -1 * sin(az);
    sonar_image.beam_directions[idx].z = cos(az);
  }

  // QUESTION(lindzey): Is this actually right?
  //    Do their ranges start at 0, or at the min range of 10 cm?
  //
  // (Aaron):  We don't actually know.  Given there's no way to
  //    set "minimum range", and it's not in the data struct, we
  //    have to assume is starts from zero, though as you say, it
  //    could actually start at an arbitrary offset.

  sonar_image.ranges.resize(num_ranges);
  for (unsigned int i = 0; i < num_ranges; i++) {
    sonar_image.ranges[i] =
        static_cast<float>(i + 0.5) * ping.ping()->rangeResolution;
  }

  // \todo  Why am I byte-swapping the data below.  Why not set
  // is_bigendian to true?
  // NOTE(lindzey): That would be a good test of all our downstream processing
  // code =)
  sonar_image.image.is_bigendian = false;
  if (ping.dataSize() == 1) {
    sonar_image.image.dtype = sonar_image.image.DTYPE_UINT8;
  } else if (ping.dataSize() == 2) {
    sonar_image.image.dtype = sonar_image.image.DTYPE_UINT16;
  } else if (ping.dataSize() == 4) {
    sonar_image.image.dtype = sonar_image.image.DTYPE_UINT32;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ping_to_sonar_image"), "Unrecognized data size: %d", ping.dataSize());
  }

  sonar_image.image.beam_count = num_bearings;

  for (unsigned int r = 0; r < num_ranges; r++) {
    for (unsigned int b = 0; b < num_bearings; b++) {
      if (ping.dataSize() == 1) {
        const uint8_t data = ping.image().at_uint8(b, r);
        sonar_image.image.data.push_back(data & 0xFF);
      } else if (ping.dataSize() == 2) {
        // Data is stored little-endian (lower byte first)
        const uint16_t data = ping.image().at_uint16(b, r);
        sonar_image.image.data.push_back(data & 0xFF);
        sonar_image.image.data.push_back((data & 0xFF00) >> 8);
      } else if (ping.dataSize() == 4) {
        // Data is stored in the sonar_image little-endian (lower byte first)
        const uint32_t data = ping.image().at_uint32(b, r);
        sonar_image.image.data.push_back(data & 0x000000FF);
        sonar_image.image.data.push_back((data & 0x0000FF00) >> 8);
        sonar_image.image.data.push_back((data & 0x00FF0000) >> 16);
        sonar_image.image.data.push_back((data & 0xFF000000) >> 24);
      }
    }
  }

  return sonar_image;
}

}  // namespace oculus_sonar_driver
