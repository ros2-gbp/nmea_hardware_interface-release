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

#ifndef NMEA_HARDWARE_INTERFACE__GEOPOSE_PUBLISHER_HPP_
#define NMEA_HARDWARE_INTERFACE__GEOPOSE_PUBLISHER_HPP_

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/controller_interface.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace nmea_hardware_interface
{
class GeoPosePublisher : public controller_interface::ControllerInterface
{
public:
  controller_interface::return_type init(const std::string & controller_name) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

#if GALACTIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init()
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

#if GALACTIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
#else
  controller_interface::return_type update() override;
#endif

private:
  double publish_rate_;
  double update_duration_;
  size_t size_;
  std::string geopose_topic_;
  std::string frame_id_;
  std::string qos_;
  bool isfirsttime;
  std::shared_ptr<rclcpp::Clock> clock_ptr_;
  geographic_msgs::msg::GeoPoseStamped geopose_;
  double getValue(const std::string & joint_name, const std::string & interface_name);
  double configure_time_;
  double next_update_time_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geographic_msgs::msg::GeoPoseStamped>>
    geopose_pub_realtime_;
};
}  // namespace nmea_hardware_interface

#endif  // NMEA_HARDWARE_INTERFACE__GEOPOSE_PUBLISHER_HPP_
