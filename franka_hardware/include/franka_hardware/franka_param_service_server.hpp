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

#pragma once

#include <memory>

#include "franka/exception.h"
#include "franka_hardware/robot.hpp"

#include <franka_msgs/srv/set_cartesian_stiffness.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_stiffness.hpp>
#include <franka_msgs/srv/set_load.hpp>
#include <franka_msgs/srv/set_stiffness_frame.hpp>
#include <franka_msgs/srv/set_tcp_frame.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * Node implementing the service server
 */
namespace franka_hardware {

class FrankaParamServiceServer : public rclcpp::Node {
 public:
  FrankaParamServiceServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot);

 private:
  /**
   * @brief generic templated param setter function
   *
   * @param param_setter_function: std::function<void(request_type)> makes the call to the robot
   * class which take the request type
   * @param request franka_msgs::srv set parameter request type
   * @param response franka_msgs::srv set parameter response type
   */
  template <typename request_type, typename response_type>
  void setGenericRobotParam(const std::function<void(request_type)>& param_setter_function,
                            const request_type& request,
                            const response_type& response) {
    try {
      param_setter_function(request);
      response->success = true;
    } catch (const franka::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during parameter setting: %s", e.what());
      response->success = false;
      response->error = e.what();
    }
  }

  /**
   * @brief Callback function for set_joint_stiffness service
   *
   * @param request shared_ptr to setJointStiffness service msgs request
   * @param response shared_ptr to setJointStiffness service msgs response
   */
  void setJointStiffnessCallback(
      const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& request,
      const franka_msgs::srv::SetJointStiffness::Response::SharedPtr& response);

  /**
   * @brief Callback function for set_cartesian_stiffness service
   *
   * @param request shared_ptr to SetCartesianStiffness service msgs request
   * @param response shared_ptr to SetCartesianStiffness service msgs response
   */
  void setCartesianStiffnessCallback(
      const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& request,
      const franka_msgs::srv::SetCartesianStiffness::Response::SharedPtr& response);

  /**
   * @brief Callback function for set_cartesian_stiffness service
   *
   * @param request shared_ptr to SetCartesianStiffness service msgs request
   * @param response shared_ptr to SetCartesianStiffness service msgs response
   */
  void setTCPFrameCallback(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& request,
                           const franka_msgs::srv::SetTCPFrame::Response::SharedPtr& response);

  /**
   * @brief Callback function for set_stiffness_frame service
   *
   * @param request shared_ptr to SetStiffnessFrame service msgs request
   * @param response shared_ptr to SetStiffnessFrame service msgs response
   */
  void setStiffnessFrameCallback(
      const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& request,
      const franka_msgs::srv::SetStiffnessFrame::Response::SharedPtr& response);

  /**
   * @brief Callback function for set_force_torque_collision_behavior service
   *
   * @param request shared_ptr to SetForceTorqueCollisionBehavior service msgs request
   * @param response shared_ptr to SetForceTorqueCollisionBehavior service msgs response
   */
  void setForceTorqueCollisionBehaviorCallback(
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& request,
      const franka_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr& response);

  /**
   * @brief Callback function for set_full_collision_behavior service
   *
   * @param request shared_ptr to SetFullCollisionBehavior service msgs request
   * @param response shared_ptr to SetFullCollisionBehavior service msgs response
   */
  void setFullCollisionBehaviorCallback(
      const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& request,
      const franka_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr& response);

  /**
   * @brief Callback function for set_load_callback service
   *
   * @param request shared_ptr to SetLoad service msgs request
   * @param response shared_ptr to SetLoad service msgs response
   */
  void setLoadCallback(const franka_msgs::srv::SetLoad::Request::SharedPtr& request,
                       const franka_msgs::srv::SetLoad::Response::SharedPtr& response);

  std::shared_ptr<Robot> robot_;

  rclcpp::Service<franka_msgs::srv::SetJointStiffness>::SharedPtr set_joint_stiffness_service_;
  rclcpp::Service<franka_msgs::srv::SetCartesianStiffness>::SharedPtr
      set_cartesian_stiffness_service_;
  rclcpp::Service<franka_msgs::srv::SetLoad>::SharedPtr set_load_service_;
  rclcpp::Service<franka_msgs::srv::SetTCPFrame>::SharedPtr set_tcp_frame_service_;
  rclcpp::Service<franka_msgs::srv::SetStiffnessFrame>::SharedPtr set_stiffness_frame_service_;
  rclcpp::Service<franka_msgs::srv::SetForceTorqueCollisionBehavior>::SharedPtr
      set_force_torque_collision_behavior_service_;
  rclcpp::Service<franka_msgs::srv::SetFullCollisionBehavior>::SharedPtr
      set_full_collision_behavior_service_;
};
}  // namespace franka_hardware
