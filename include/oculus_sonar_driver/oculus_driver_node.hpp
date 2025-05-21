// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey
// ROS2 port

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>

#include "liboculus/IoServiceThread.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarConfiguration.h"
#include "liboculus/StatusRx.h"
#include "oculus_sonar_driver/ping_to_sonar_image.h"
#include "oculus_sonar_driver/publishing_data_rx.h"

// Only keep marine_acoustic_msgs
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"

namespace oculus_sonar_driver {

class OculusDriver : public rclcpp::Node {
 public:
  OculusDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OculusDriver();

  // Translate SimplePingResult to SonarImage and publish
  template <typename Ping_t>
  void pingCallback(const Ping_t &ping) {
    // Publish message parsed into the image format
    marine_acoustic_msgs::msg::ProjectedSonarImage sonar_msg =
        pingToSonarImage(ping);

    sonar_msg.header.stamp = this->now();
    sonar_msg.header.frame_id = frame_id_;
    imaging_sonar_pub_->publish(sonar_msg);
  }

  // Update configuration based on parameter change
  rcl_interfaces::msg::SetParametersResult configCallback(const std::vector<rclcpp::Parameter> &parameters);

 private:
  void setupPublishers();
  void setupParameters();

  // Use correct type for parameter callback handle
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_callback_handle_;

  liboculus::IoServiceThread io_srv_;
  liboculus::StatusRx status_rx_;
  PublishingDataRx data_rx_;

  rclcpp::Publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr imaging_sonar_pub_;

  std::string ip_address_;
  std::string frame_id_;

  liboculus::SonarConfiguration sonar_config_;
};

}  // namespace oculus_sonar_driver
