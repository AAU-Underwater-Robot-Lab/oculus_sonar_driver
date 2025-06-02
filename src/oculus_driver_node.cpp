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

  // Set up data_rx_ callbacks for SimplePingResultV1 and V2
  data_rx_.setCallback<liboculus::SimplePingResultV1>(
      std::bind(&OculusDriver::pingCallback<liboculus::SimplePingResultV1>, this, std::placeholders::_1));
  data_rx_.setCallback<liboculus::SimplePingResultV2>(
      std::bind(&OculusDriver::pingCallback<liboculus::SimplePingResultV2>, this, std::placeholders::_1));

  // Set up on-connect callback to send current config
  data_rx_.setOnConnectCallback([
    this
  ]() {
    data_rx_.sendSimpleFireMessage(sonar_config_);
  });

  // Set up raw data publisher if/when available
  // data_rx_.setRawPublisher(raw_data_pub_);

  // Handle auto IP detection or direct connect
  if (ip_address_ == "auto") {
    RCLCPP_INFO(this->get_logger(), "Attempting to auto-detect sonar");
    status_rx_.setCallback([
      this
    ](const liboculus::SonarStatus &status, bool is_valid) {
      if (!is_valid || data_rx_.isConnected()) return;
      RCLCPP_WARN(this->get_logger(), "Auto-detected IP: %s", status.ipAddr().c_str());
      data_rx_.connect(status.ipAddr());
    });
  } else {
    RCLCPP_INFO(this->get_logger(), "Opening sonar at %s", ip_address_.c_str());
    data_rx_.connect(ip_address_);
  }

  io_srv_.start();
}

OculusDriver::~OculusDriver() {
  io_srv_.stop();
  io_srv_.join();
}

void OculusDriver::setupParameters() {
  this->declare_parameter<bool>("send_range_as_meters", true);
  this->declare_parameter<bool>("send_gain", true);
  this->declare_parameter<bool>("send_simple_return", true);
  this->declare_parameter<bool>("gain_assistance", false);

  // num_beams: 0=256, 1=512 (default 1)
  this->declare_parameter<int>("num_beams", 1);
  // range: default 2, min 0, max 40
  this->declare_parameter<double>("range", 2.0);
  // gain: default 1, min 1, max 100
  this->declare_parameter<double>("gain", 1.0);
  // gamma: default 127, min 0, max 255
  this->declare_parameter<int>("gamma", 127);

  // ping_rate: 0=Normal, 1=High, 2=Highest, 3=Low, 4=Lowest, 5=Standby (default 0)
  this->declare_parameter<int>("ping_rate", 0);
  // data_size: 1=8bit, 2=16bit, 4=32bit (default 1)
  this->declare_parameter<int>("data_size", 1);
  // freq_mode: 1=LowFrequency, 2=HighFrequency (default 2)
  this->declare_parameter<int>("freq_mode", 2);

  this->declare_parameter<std::string>("ip_address", "192.168.1.111");
  this->declare_parameter<std::string>("frame_id", "sonar");

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
  double range = this->get_parameter("range").as_double();
  double gain = this->get_parameter("gain").as_double();
  int gamma = this->get_parameter("gamma").as_int();
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

  // Map ping_rate to liboculus enums
  switch (ping_rate) {
    case 0: sonar_config_.setPingRate(pingRateNormal); break;
    case 1: sonar_config_.setPingRate(pingRateHigh); break;
    case 2: sonar_config_.setPingRate(pingRateHighest); break;
    case 3: sonar_config_.setPingRate(pingRateLow); break;
    case 4: sonar_config_.setPingRate(pingRateLowest); break;
    case 5: sonar_config_.setPingRate(pingRateStandby); break;
    default: RCLCPP_WARN(this->get_logger(), "Unknown ping rate %d", ping_rate);
  }

  // Map freq_mode to liboculus enums
  switch (freq_mode) {
    case 1: sonar_config_.setFreqMode(liboculus::OCULUS_LOW_FREQ); break;
    case 2: sonar_config_.setFreqMode(liboculus::OCULUS_HIGH_FREQ); break;
    default: RCLCPP_WARN(this->get_logger(), "Unknown frequency mode %d", freq_mode);
  }

  sonar_config_.sendRangeAsMeters(send_range_as_meters)
      .setSendGain(send_gain)
      .setSimpleReturn(send_simple_return)
      .setGainAssistance(gain_assistance);

  // num_beams: 0=256, 1=512
  if (num_beams == 0) {
    sonar_config_.use256Beams();
  } else {
    sonar_config_.use512Beams();
  }

  // data_size: 1=8bit, 2=16bit, 4=32bit
  switch (data_size) {
    case 1: sonar_config_.setDataSize(dataSize8Bit); break;
    case 2: sonar_config_.setDataSize(dataSize16Bit); break;
    case 4: sonar_config_.setDataSize(dataSize32Bit); break;
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
