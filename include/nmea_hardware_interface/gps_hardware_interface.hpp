// Copyright (c) 2021 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NMEA_HARDWARE_INTERFACE__GPS_HARDWARE_INTERFACE_HPP_
#define NMEA_HARDWARE_INTERFACE__GPS_HARDWARE_INTERFACE_HPP_

// Headers in ROS
#include <quaternion_operation/quaternion_operation.h>

#if GALACTIC
#include <hardware_interface/system_interface.hpp>
#else
#include <hardware_interface/base_interface.hpp>
#endif
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#if GALACTIC
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#else
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#endif
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geographic_msgs/msg/geo_pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <nmea_msgs/msg/sentence.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace nmea_hardware_interface
{
class GPSHardwareInterface
#if GALACTIC
: public hardware_interface::SensorInterface
#else
: public hardware_interface::BaseInterface<hardware_interface::SensorInterface>
#endif
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GPSHardwareInterface)

#if GALACTIC

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#else

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
#endif

  ~GPSHardwareInterface();

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

#ifndef GALACTIC

  hardware_interface::return_type start() override;

  hardware_interface::return_type stop() override;
#endif

  hardware_interface::return_type read() override;

private:
  std::string joint_;
  nmea_msgs::msg::Sentence sentence_;
  size_t size_;

  boost::thread togeopose_thread_;
  std::string device_file_;
  int baud_rate_;
  std::string frame_id_;
  boost::asio::io_service io_;
  std::shared_ptr<boost::asio::serial_port> port_ptr_;
  boost::thread io_thread_;
  void readSentence();
  boost::array<char, 256> buf_;

  void connectSerialPort();
  bool connected_ = false;
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  boost::optional<std::string> validate(std::string sentence);
  bool validatecheckSum(std::string sentence);
  std::string getHexString(uint8_t value);

  void nmea_to_geopose();
  std::string calculateChecksum(std::string sentence);
  geographic_msgs::msg::GeoPoint geopoint_;
  geometry_msgs::msg::Quaternion quat_;
  geographic_msgs::msg::GeoPose geopose_;
  bool isGprmcSentence(nmea_msgs::msg::Sentence sentence);
  bool isGphdtSentence(nmea_msgs::msg::Sentence sentence);
  std::vector<std::string> split(const std::string & s, char delim);
  std::vector<std::string> splitChecksum(std::string str);
  boost::optional<std::vector<std::string>> splitSentence(nmea_msgs::msg::Sentence sentence);
};
}  // namespace nmea_hardware_interface

#endif  // NMEA_HARDWARE_INTERFACE__GPS_HARDWARE_INTERFACE_HPP_
