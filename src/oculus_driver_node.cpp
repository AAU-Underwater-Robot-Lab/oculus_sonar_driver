// Copyright 2020-2022 UW-APL
// Authors: Aaron Marburg, Laura Lindzey
// ROS2 port

#include "oculus_sonar_driver/oculus_driver_node.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace oculus_sonar_driver {

OculusDriver::OculusDriver(const rclcpp::NodeOptions & options)
    : rclcpp::Node("oculus_driver", options),
      io_srv_(),
      data_rx_(io_srv_.context()),
      status_rx_(io_srv_.context()) {
  setupParameters();
  setupPublishers();
  // TODO: Set up data_rx_ and status_rx_ as needed for ROS2
}

OculusDriver::~OculusDriver() {}

void OculusDriver::setupParameters() {
  this->declare_parameter<std::string>("ip_address", "192.168.1.111");
  this->declare_parameter<std::string>("frame_id", "sonar");
  this->declare_parameter<double>("range", 10.0);
  this->declare_parameter<int>("gain", 50);
  this->declare_parameter<double>("gamma", 1.0);
  this->declare_parameter<int>("ping_rate", 0);
  this->declare_parameter<int>("freq_mode", 0);
  this->declare_parameter<bool>("send_range_as_meters", false);
  this->declare_parameter<bool>("send_gain", false);
  this->declare_parameter<bool>("send_simple_return", false);
  this->declare_parameter<bool>("gain_assistance", false);
  this->declare_parameter<int>("num_beams", 512);
  this->declare_parameter<int>("data_size", 8);

  this->get_parameter("ip_address", ip_address_);
  this->get_parameter("frame_id", frame_id_);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> &params) {
      return this->configCallback(params);
    });
}

void OculusDriver::setupPublishers() {
  imaging_sonar_pub_ = this->create_publisher<marine_acoustic_msgs::msg::ProjectedSonarImage>(
      "sonar_image", rclcpp::SensorDataQoS());
}

rcl_interfaces::msg::SetParametersResult OculusDriver::configCallback(const std::vector<rclcpp::Parameter> &parameters) {
  // We'll use the current parameter values for all fields
  double range = this->get_parameter("range").as_double();
  int gain = this->get_parameter("gain").as_int();
  double gamma = this->get_parameter("gamma").as_double();
  int ping_rate = this->get_parameter("ping_rate").as_int();
  int freq_mode = this->get_parameter("freq_mode").as_int();
  bool send_range_as_meters = this->get_parameter("send_range_as_meters").as_bool();
  bool send_gain = this->get_parameter("send_gain").as_bool();
  bool send_simple_return = this->get_parameter("send_simple_return").as_bool();
  bool gain_assistance = this->get_parameter("gain_assistance").as_bool();
  int num_beams = this->get_parameter("num_beams").as_int();
  int data_size = this->get_parameter("data_size").as_int();

  sonar_config_.setRange(range);
  sonar_config_.setGainPercent(gain);
  sonar_config_.setGamma(gamma);

  // Map ping_rate, freq_mode, num_beams, data_size to liboculus enums as in ROS1
  switch (ping_rate) {
    case 0: sonar_config_.setPingRate(pingRateNormal); break;
    case 1: sonar_config_.setPingRate(pingRateHigh); break;
    case 2: sonar_config_.setPingRate(pingRateHighest); break;
    case -1: sonar_config_.setPingRate(pingRateLow); break;
    case -2: sonar_config_.setPingRate(pingRateLowest); break;
    case 99: sonar_config_.setPingRate(pingRateStandby); break;
    default: RCLCPP_WARN(this->get_logger(), "Unknown ping rate %d", ping_rate);
  }

  switch (freq_mode) {
    case 0: sonar_config_.setFreqMode(liboculus::OCULUS_LOW_FREQ); break;
    case 1: sonar_config_.setFreqMode(liboculus::OCULUS_HIGH_FREQ); break;
    default: RCLCPP_WARN(this->get_logger(), "Unknown frequency mode %d", freq_mode);
  }

  sonar_config_.sendRangeAsMeters(send_range_as_meters)
      .setSendGain(send_gain)
      .setSimpleReturn(send_simple_return)
      .setGainAssistance(gain_assistance);

  if (num_beams == 256) {
    sonar_config_.use256Beams();
  } else {
    sonar_config_.use512Beams();
  }

  switch (data_size) {
    case 8: sonar_config_.setDataSize(dataSize8Bit); break;
    case 16: sonar_config_.setDataSize(dataSize16Bit); break;
    case 32: sonar_config_.setDataSize(dataSize32Bit); break;
    default: RCLCPP_WARN(this->get_logger(), "Unknown data size %d", data_size);
  }

  // Update the sonar with new params
  if (data_rx_.isConnected()) {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";
  return result;
}

// Explicit template instantiation if needed
// template void oculus_sonar_driver::OculusDriver::pingCallback<YourPingType>(const YourPingType &);

}  // namespace oculus_sonar_driver

// Add main() for ROS2 node
#include "rclcpp/rclcpp.hpp"
#include "oculus_sonar_driver/oculus_driver_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<oculus_sonar_driver::OculusDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
