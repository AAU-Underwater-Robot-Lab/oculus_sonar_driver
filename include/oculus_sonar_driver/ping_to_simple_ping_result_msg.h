// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#pragma once

#include "blueprint_oculus_msgs/OculusSimplePingResultMsg.h"
#include "liboculus/Constants.h"
#include "liboculus/SimplePingResult.h"
#include "ros/ros.h"

namespace oculus_sonar_driver {

// This templated function handles the fields that are common to both the
//  SimplePingResult and SimplePingResultV2 headers.
//
template <typename PingT>
blueprint_oculus_msgs::OculusSimplePingResultMsg pingToPingResultCommon(
    const liboculus::SimplePingResult<PingT> &ping) {
  blueprint_oculus_msgs::OculusSimplePingResultMsg ping_result;

  // Fields from OculusMessageHeader
  ping_result.src_device_id = ping.ping()->fireMessage.head.srcDeviceId;
  ping_result.dst_device_id = ping.ping()->fireMessage.head.dstDeviceId;
  ping_result.msg_id = ping.ping()->fireMessage.head.msgId;
  ping_result.msg_version = ping.ping()->fireMessage.head.msgVersion;
  ping_result.payload_size = ping.ping()->fireMessage.head.payloadSize;
  // ping_result.spare2 = ping.ping()->fireMessage.head.spare2;

  // ## Fields from OculusSimpleFireMessage / OculusSimpleFireMessage2
  ping_result.master_mode = ping.ping()->fireMessage.masterMode;
  // \todo Define ROS Msg ENUMs for the ping rate types
  ping_result.ping_rate = ping.ping()->fireMessage.pingRate;
  ping_result.network_speed = ping.ping()->fireMessage.networkSpeed;
  ping_result.gamma_correction = ping.ping()->fireMessage.gammaCorrection;
  ping_result.flags = ping.ping()->fireMessage.flags;
  ping_result.gain_percent = ping.ping()->fireMessage.gainPercent;
  ping_result.speed_of_sound = ping.ping()->fireMessage.speedOfSound;
  ping_result.salinity = ping.ping()->fireMessage.salinity;

  // Fields from OculusSimplePingResult / OculusSimplePingResult2
  ping_result.ping_id = ping.ping()->pingId;
  ping_result.status = ping.ping()->status;
  ping_result.frequency = ping.ping()->frequency;
  ping_result.temperature = ping.ping()->temperature;
  ping_result.pressure = ping.ping()->pressure;
  ping_result.speed_of_sound_used = ping.ping()->speedOfSoundUsed;
  ping_result.ping_start_time = ping.ping()->pingStartTime;

  // \todo Define ROS Msg ENUMs for the data size types?
  ping_result.data_size = ping.ping()->dataSize;
  ping_result.range_resolution = ping.ping()->rangeResolution;

  ping_result.n_ranges = ping.ping()->nRanges;
  ping_result.n_beams = ping.ping()->nBeams;

  // uint32 spare0 uint32 spare1 uint32 spare2 uint32 spare3
  ping_result.image_offset = ping.ping()->imageOffset;
  ping_result.image_size = ping.ping()->imageSize;
  ping_result.message_size = ping.ping()->messageSize;

  return ping_result;
}

// Builds a OculusSimplePingResultMsg ROS message from the contents of a
// liboculus::SimplePingResult header
//
// General template case.  Never used -- we have specializations for the two
// potential types (below)
//
template <typename PingT>
blueprint_oculus_msgs::OculusSimplePingResultMsg pingToPingResult(
    const liboculus::SimplePingResult<PingT> &ping) {
  return pingToPingResultCommon(ping);
}

// Specialization for SimplePingResultV1
// Handles extra fields that don't exist in SimplePingResultV2
//
template <>
blueprint_oculus_msgs::OculusSimplePingResultMsg
pingToPingResult<OculusSimplePingResult>(
    const liboculus::SimplePingResult<OculusSimplePingResult> &ping) {
  auto ping_result = pingToPingResultCommon(ping);

  ping_result.range = ping.ping()->fireMessage.range;

  return ping_result;
}

// Specialization for SimplePingResultV2
// Handles extra fields that don't exist in SimplePingResultV1
//
template <>
blueprint_oculus_msgs::OculusSimplePingResultMsg
pingToPingResult<OculusSimplePingResult2>(
    const liboculus::SimplePingResultV2 &ping) {
  auto ping_result = pingToPingResultCommon(ping);
  ping_result.range = ping.ping()->fireMessage.rangePercent;
  ping_result.ext_flags = ping.ping()->fireMessage.extFlags;

  ping_result.heading = ping.ping()->heading;
  ping_result.pitch = ping.ping()->pitch;
  ping_result.roll = ping.ping()->roll;

  return ping_result;
}

}  // namespace oculus_sonar_driver
