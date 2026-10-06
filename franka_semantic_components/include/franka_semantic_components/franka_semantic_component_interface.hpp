// Copyright (c) 2023 Franka Robotics GmbH
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

// modified ros2 control semantic control interface to add command interface access.
// https://github.com/ros-controls/ros2_control/blob/humble/controller_interface/include/semantic_components/semantic_component_interface.hpp

#pragma once

#include <array>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace franka_semantic_components {

class FrankaSemanticComponentInterface {
 public:
  /**
   * \param[in] name of the semantic component interface. Used to prefix the state and command
   * interfaces if not assigned
   *
   * \param[in] state_interface_size size of the loaned state interfaces. If not given defaulted to
   * 0.
   *
   * \param[in] command_interface_size size of the loaned command interfaces. If not given defaulted
   * to 0.
   */
  explicit FrankaSemanticComponentInterface(const std::string& name,
                                            size_t state_interface_size = 0,
                                            size_t command_interface_size = 0);
  FrankaSemanticComponentInterface(const FrankaSemanticComponentInterface&) = delete;
  FrankaSemanticComponentInterface& operator=(FrankaSemanticComponentInterface const&) = delete;
  FrankaSemanticComponentInterface(FrankaSemanticComponentInterface&&) = default;
  FrankaSemanticComponentInterface& operator=(FrankaSemanticComponentInterface&&) = default;

  virtual ~FrankaSemanticComponentInterface() = default;

  /**
   * Assign loaned state interfaces from the hardware.Assign loaned state interfaces on the
   * controller start.
   *
   * \param[in] state_interfaces vector of interfaces provided by the controller.
   * \return true when success, else false
   */
  bool assign_loaned_state_interfaces(
      std::vector<hardware_interface::LoanedStateInterface>& state_interfaces);

  /**
   * Assign loaned command interfaces from the hardware. Assign loaned command interfaces on
   * the controller start.
   *
   * \param[in] command_interfaces vector of interfaces provided by the controller.
   * \return true when success, else false
   */
  bool assign_loaned_command_interfaces(
      std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces);

  /// Release loaned interfaces from the hardware.
  void release_interfaces();

  /**
   * The function should be used in "state_interface_configuration()" of a controller to provide
   * standardized interface names for the semantic component.
   *
   * \default Default implementation defined state interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with state interface names for the semantic component.
   */
  virtual std::vector<std::string> get_state_interface_names();

  /**
   * Definition of command interface names for the component.
   *
   * The function should be used in "command_interface_configuration()" of a controller to provide
   * standardized interface names semantic component.
   *
   * \default Default implementation defined command interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with command interface names for the semantic component.
   */
  virtual std::vector<std::string> get_command_interface_names();

  /**
   * Return all values of the state interfaces.
   *
   * \return std::vector<double>  state_interface_values
   */
  [[nodiscard]] std::vector<double> get_values_state_interfaces() const;

  /**
   *  Return all values for the command interfaces
   *
   * \return std::vector<double>  command_interface_values
   */
  [[nodiscard]] std::vector<double> get_values_command_interfaces() const;

  /**
   * Set all values for the command interfaces
   *
   * \param[in] std::vector<double> commanded values to be set
   *
   * \return true if commanded_values size matches the loaned command_interface size, else false
   */
  bool set_values(const std::vector<double>& commanded_values);

  /**
   * Set all values for the command interfaces without allocating.
   *
   * Stack-storage alternative to set_values(const std::vector<double>&) for the real-time
   * path: the caller passes a fixed-size array, so no heap buffer is built per call.
   *
   * \param[in] commanded_values to be set; its size must match the loaned command interfaces
   *
   * \return true if the array size matches the loaned command_interface size, else false
   */
  template <size_t NumValues>
  bool set_values(const std::array<double, NumValues>& commanded_values) {
    if (NumValues != command_interfaces_.size()) {
      return false;
    }
    bool success = true;
    for (size_t index = 0; index < NumValues; ++index) {
      success &= command_interfaces_[index].get().set_value(commanded_values[index]);
    }
    return success;
  }

 protected:
  std::string name_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,
                      // misc-non-private-member-variables-in-classes)
  std::vector<std::string>
      state_interface_names_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,
                               // misc-non-private-member-variables-in-classes)
  std::vector<std::string>
      command_interface_names_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,
                                 // misc-non-private-member-variables-in-classes)

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      state_interfaces_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,
                          // misc-non-private-member-variables-in-classes)
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      command_interfaces_;  // NOLINT(cppcoreguidelines-non-private-member-variables-in-classes,
                            // misc-non-private-member-variables-in-classes)
};

}  // namespace franka_semantic_components
