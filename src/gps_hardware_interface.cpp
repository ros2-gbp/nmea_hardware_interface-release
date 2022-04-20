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

#include "nmea_hardware_interface/gps_hardware_interface.hpp"

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace nmea_hardware_interface
{
#if GALACTIC
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GPSHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
#else
hardware_interface::return_type GPSHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
#endif
{
  device_file_ = info.hardware_parameters.at("device_file");
  baud_rate_ = std::stoi(info.hardware_parameters.at("baud_rate"));
  frame_id_ = info.hardware_parameters.at("frame_id");
  connectSerialPort();
  using namespace std::chrono_literals;
// timer_ = rclcpp::create_wall_timer(
//        1000ms, std::bind(&GPSHardwareInterface::timerCallback, this));
#if GALACTIC
  if (
    SensorInterface::on_init(info) !=
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
#else
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
#endif
  if (info.joints.size() != 1) {
    throw std::runtime_error("joint size should be 1");
  }
  joint_ = info.joints[0].name;

#if GALACTIC
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
#else
  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
#endif
}

GPSHardwareInterface::~GPSHardwareInterface()
{
  io_thread_.join();
  togeopose_thread_.join();
}

std::vector<hardware_interface::StateInterface> GPSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces = {};
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("nmea_gps", "latitude", &geopose_.position.latitude));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("nmea_gps", "longitude", &geopose_.position.longitude));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("nmea_gps", "altitude", &geopose_.position.altitude));

  return state_interfaces;
}

#ifndef GALACTIC
hardware_interface::return_type GPSHardwareInterface::start()
{
  status_ = hardware_interface::status::STARTED;
  togeopose_thread_ = boost::thread(boost::bind(&GPSHardwareInterface::nmea_to_geopose, this));
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GPSHardwareInterface::stop()
{
  io_thread_.join();
  togeopose_thread_.join();
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}
#endif

hardware_interface::return_type GPSHardwareInterface::read()
{
  return hardware_interface::return_type::OK;
}

bool GPSHardwareInterface::validatecheckSum(std::string sentence)
{
  char delim = '*';

  auto splited_sentence = split(sentence, delim);
  if (splited_sentence.size() != 2) {
    return false;
  }
  uint8_t checksum = 0;
  std::string xor_target_str = splited_sentence[0].substr(1, splited_sentence[0].size() - 1);
  for (int i = 0; i < static_cast<int>(xor_target_str.size()); i++) {
    int c = xor_target_str[i];
    checksum ^= c;
  }
  uint8_t rest = checksum % 16;
  uint8_t quotient = (checksum - rest) / 16;
  std::string ret = getHexString(quotient) + getHexString(rest);
  if (ret == splited_sentence[1]) {
    return true;
  }
  std::string message = "checksum does not match in calculating sentence :" + sentence +
                        " calculated checksum is " + ret;
  return false;
}

std::string GPSHardwareInterface::getHexString(uint8_t value)
{
  std::string ret;
  if (value == 10) {
    ret = "A";
  } else if (value == 11) {
    ret = "B";
  } else if (value == 12) {
    ret = "C";
  } else if (value == 13) {
    ret = "D";
  } else if (value == 14) {
    ret = "E";
  } else if (value == 15) {
    ret = "F";
  } else {
    ret = std::to_string(value);
  }
  return ret;
}

boost::optional<std::string> GPSHardwareInterface::validate(std::string sentence)
{
  try {
    sentence = "$" + sentence;
    std::stringstream ss1{sentence};
    if (std::getline(ss1, sentence)) {
      std::stringstream ss2{sentence};
      if (std::getline(ss2, sentence, '\r')) {
        if (validatecheckSum(sentence)) {
          return sentence;
        }
        return boost::none;
      }
    }
  } catch (const std::exception & e) {
    std::string message = "while processing : " + sentence + " : " + e.what();
    return boost::none;
  }
  return sentence;
}

void GPSHardwareInterface::connectSerialPort()
{
  try {
    port_ptr_ = std::make_shared<boost::asio::serial_port>(io_, device_file_);

    port_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));

    port_ptr_->set_option(boost::asio::serial_port_base::character_size(8));

    port_ptr_->set_option(boost::asio::serial_port_base::flow_control(
      boost::asio::serial_port_base::flow_control::none));

    port_ptr_->set_option(
      boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));

    port_ptr_->set_option(
      boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    io_thread_ = boost::thread(boost::bind(&GPSHardwareInterface::readSentence, this));

    connected_ = true;
  } catch (const std::exception & e) {
    std::cout << e.what() << std::endl;

    connected_ = false;
  }
}

void GPSHardwareInterface::timerCallback()
{
  if (!connected_) {
    connectSerialPort();
  }
}

std::vector<std::string> GPSHardwareInterface::split(const std::string & s, char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(s);
  std::string item;
  while (getline(ss, item, delim)) {
    if (!item.empty()) {
      elems.push_back(item);
    }
  }
  return elems;
}

void GPSHardwareInterface::readSentence()
{
  while (rclcpp::ok()) {
    buf_ = boost::array<char, 256>();
    try {
      port_ptr_->read_some(boost::asio::buffer(buf_));
    } catch (const std::exception & e) {
      connected_ = false;
      return;
    }
    std::string data(buf_.begin(), buf_.end());
    std::vector<std::string> splited_sentence = split(data, '$');

    for (auto itr = splited_sentence.begin(); itr != splited_sentence.end(); itr++) {
      auto line = validate(*itr);
      if (line) {
        sentence_.header.frame_id = frame_id_;
        sentence_.sentence = line.get();
        nmea_to_geopose();
      }
    }
  }
}

void GPSHardwareInterface::nmea_to_geopose()
{
  if (isGprmcSentence(sentence_)) {
    geographic_msgs::msg::GeoPoint geopoint;
    boost::optional<std::vector<std::string>> data = splitSentence(sentence_);

    if (data) {
      std::string lat_str = data.get()[3];
      std::string north_or_south_str = data.get()[4];
      double latitude = std::stod(lat_str.substr(0, 2)) + std::stod(lat_str.substr(2)) / 60.0;
      assert(north_or_south_str == "N" || north_or_south_str == "S");
      if (north_or_south_str == "S") {
        latitude = latitude * -1;
      }
      std::string lon_str = data.get()[5];
      std::string east_or_west_str = data.get()[6];
      double longitude = std::stod(lon_str.substr(0, 3)) + std::stod(lon_str.substr(3)) / 60.0;
      assert(east_or_west_str == "E" || east_or_west_str == "W");
      if (east_or_west_str == "W") {
        longitude = longitude * -1;
      }
      geopoint.latitude = latitude;
      geopoint.longitude = longitude;
      geopoint.altitude = 0.0;
      geopoint_ = geopoint;
    }
  }
  if (isGphdtSentence(sentence_)) {
    boost::optional<std::vector<std::string>> data = splitSentence(sentence_);
    if (data) {
      if (data.get()[2] == "T") {
        double heading = std::stod(data.get()[1]);
        geometry_msgs::msg::Vector3 vec;
        vec.x = 0.0;
        vec.y = 0.0;
        vec.z = heading / 180 * M_PI * -1;
        geometry_msgs::msg::Quaternion quat =
          quaternion_operation::convertEulerAngleToQuaternion(vec);
        quat_ = quat;
      }
    }
  }
  geographic_msgs::msg::GeoPose geopose;
  geopose.position = geopoint_;
  geopose.orientation = quat_;
  geopose_ = geopose;
}

std::vector<std::string> GPSHardwareInterface::splitChecksum(std::string str)
{
  return split(str, '*');
}

boost::optional<std::vector<std::string>> GPSHardwareInterface::splitSentence(
  nmea_msgs::msg::Sentence sentence)
{
  std::vector<std::string> data = splitChecksum(sentence.sentence);

  if (data.size() != 2) {
    return boost::none;
  }
  if (calculateChecksum(data[0]) == data[1]) {
    return split(data[0], ',');
  }

  return boost::none;
}

bool GPSHardwareInterface::isGprmcSentence(nmea_msgs::msg::Sentence sentence)
{
  std::string type = sentence.sentence.substr(0, 6);
  if (type == /*"$GPRMC"*/ "$GNRMC") {
    return true;
  }
  return false;
}

bool GPSHardwareInterface::isGphdtSentence(nmea_msgs::msg::Sentence sentence)
{
  std::string type = sentence.sentence.substr(0, 6);
  if (type == "$GPHDT") {
    return true;
  }
  return false;
}

std::string GPSHardwareInterface::calculateChecksum(std::string sentence)
{
  uint8_t checksum;
  for (unsigned int i = 1; i < sentence.size(); i++) {
    int16_t c = sentence[i];
    checksum ^= c;
  }
  uint8_t rest = checksum % 16;
  uint8_t quotient = (checksum - rest) / 16;
  std::string ret = getHexString(quotient) + getHexString(rest);
  return ret;
}

}  // namespace nmea_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  nmea_hardware_interface::GPSHardwareInterface, hardware_interface::SensorInterface)
