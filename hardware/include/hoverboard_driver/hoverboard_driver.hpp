// Copyright 2021 ros2_control Development Team
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

#ifndef HOVERBOARD_DRIVER_HPP_
#define HOVERBOARD_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hoverboard_driver/config.hpp"
#include "hoverboard_driver/protocol.hpp"
#include "hoverboard_driver/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace hoverboard_driver
{

  enum
  {
    /// @brief left wheel
    left_wheel,
    /// @brief right wheel
    right_wheel
  };

  /// @brief Helper class to get a node interface. This class is used to build publishers and dynamic parameters
  /// for hoverboard_driver class.
  class hoverboard_driver_node : public rclcpp::Node
  {
  public:
    hoverboard_driver_node();
    /// @brief publish velocity data for debugging
    /// @param wheel left or right wheel
    /// @param message value to publish
    void publish_vel(int wheel, double message);

    /// @brief publish pose data for debugging
    /// @param wheel left or right wheel
    /// @param message value to publish
    void publish_pos(int wheel, double message);

    /// @brief publish command data for debugging
    /// @param wheel left or right wheel
    /// @param message value to publish
    void publish_cmd(int wheel, double message);

    /// @brief publish battery voltage
    /// @param message value to publish
    void publish_voltage(double message);

    /// @brief publish current (power consumption) of motor
    /// @param wheel left or right wheel
    /// @param message value to publish
    void publish_curr(int wheel, double message);

    /// @brief publish PCB temperature
    /// @param message value to publish
    void publish_temp(double message);


    /// @brief publish state of PCB (on or off)
    /// @param message value to publish
    void publish_connected(bool message);

    /// @brief parameter callback method. 
    /// @param parameters 
    /// @return 
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

    /// @brief PID configuration structure
    struct
    {
      double p;
      double i;
      double d;
      double f;
      double i_clamp_min;
      double i_clamp_max;
      bool antiwindup;
    } pid_config;

  private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_pub[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curr_pub[2];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub;

        // Parameter Callback handle
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  };

  /// @brief Hardware Interface class to communicate with Hoverboard PCB
  class hoverboard_driver : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(hoverboard_driver);

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    // HOVERBOARD_DRIVER_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    std::shared_ptr<hoverboard_driver_node> hardware_publisher; // make the publisher node a member
  private:
    // Store the command for the simulated robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;

    void protocol_recv(const rclcpp::Time &time, char c);
    void on_encoder_update(const rclcpp::Time &time, int16_t right, int16_t left);

    double wheel_radius;
    double max_velocity = 0.0;
    int direction_correction = 1;
    std::string port;

    rclcpp::Time last_read;
    bool first_read_pass_;
    // Last known encoder values
    int16_t last_wheelcountR;
    int16_t last_wheelcountL;
    // Count of full encoder wraps
    int multR;
    int multL;
    // Thresholds for calculating the wrap
    int low_wrap;
    int high_wrap;

    // Hoverboard protocol
    int port_fd;
    unsigned long msg_len = 0;
    char prev_byte = 0;
    uint16_t start_frame = 0;
    char *p;
    SerialFeedback msg, prev_msg;

    PID pids[2];
  };

} // namespace hoverboard_driver

#endif // HOVERBOARD_DRIVER_HPP_
