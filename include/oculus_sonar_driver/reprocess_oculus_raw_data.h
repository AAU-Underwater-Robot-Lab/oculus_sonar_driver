// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey

#pragma once

#include "apl_msgs/RawData.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"

namespace oculus_sonar_driver {

class ReprocessOculusRawData : public nodelet::Nodelet {
 public:
  ReprocessOculusRawData();
  virtual ~ReprocessOculusRawData();

  // Translate SimplePingResult to ProjectedSonarImage and publish
  void rawDataCallback(const apl_msgs::RawData::ConstPtr &raw_data);

 private:
  // Set up all ROS interfaces and start the sonarClient
  void onInit() override;

  ros::Publisher sonar_image_pub_;
  ros::Subscriber raw_data_sub_;
};

}  // namespace oculus_sonar_driver
