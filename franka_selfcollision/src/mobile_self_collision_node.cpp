// Copyright (c) 2026 Franka Robotics GmbH
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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <exception>
#include <rclcpp/parameter_client.hpp>
#include <set>
#include <sstream>

#include "franka_selfcollision/self_collision_node.hpp"

using namespace std::chrono_literals;

namespace franka_selfcollision {

// Only joints relevant to arm self-collision are tracked.
// Base, caster, rocker, and TMR drivetrain joints are excluded —
// they are kept at their neutral configuration permanently.
static const std::set<std::string> kTrackedJoints = {
    "franka_spine_vertical_joint", "left_fr3v2_joint1",  "left_fr3v2_joint2",  "left_fr3v2_joint3",
    "left_fr3v2_joint4",           "left_fr3v2_joint5",  "left_fr3v2_joint6",  "left_fr3v2_joint7",
    "right_fr3v2_joint1",          "right_fr3v2_joint2", "right_fr3v2_joint3", "right_fr3v2_joint4",
    "right_fr3v2_joint5",          "right_fr3v2_joint6", "right_fr3v2_joint7",
};

// Geometry-level filter passed to SelfCollisionChecker — keeps only collision
// pairs where both geometries belong to tracked links (arm + spine).
// Avoids false positives from base/caster/TMR links absent in the SRDF.
static bool isTrackedLink(const std::string& geom_name) {
  static const std::vector<std::string> kTrackedPrefixes = {
      "franka_spine",
      "left_fr3v2_link",
      "right_fr3v2_link",
      "duo_mount_origin",
  };
  for (const auto& prefix : kTrackedPrefixes) {
    if (geom_name.rfind(prefix, 0) == 0) {
      return true;
    }
  }
  return false;
}

CollisionMonitorNode::CollisionMonitorNode(const rclcpp::NodeOptions& options)
    : Node("self_collision_monitor", options) {
  this->declare_parameter("security_margin", 0.045);
  this->declare_parameter("print_collisions", false);
  this->declare_parameter("robot_description_semantic", "");

  collision_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "mobile_fr3_duo_self_collision_node/collision_detected", 1);
}

void CollisionMonitorNode::setup_collision_monitor(const std::string& robot_description) {
  double security_margin = this->get_parameter("security_margin").as_double();
  print_collisions_ = this->get_parameter("print_collisions").as_bool();
  std::string srdf_xml = this->get_parameter("robot_description_semantic").as_string();
  std::string urdf_xml = robot_description;

  if (urdf_xml.empty() || srdf_xml.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameters 'robot_description' (URDF) or 'robot_description_semantic' (SRDF) "
                 "are empty.");
    throw std::runtime_error("Missing XML descriptions");
  }

  RCLCPP_INFO(this->get_logger(), "Loading robot model...");

  try {
    // Pass isTrackedLink to restrict collision checking to arm + spine geometries.
    // The mobile URDF includes many base/drivetrain links absent from the SRDF;
    // without this filter they would produce phantom collisions.
    collision_checker_ = std::make_shared<franka_selfcollision::SelfCollisionChecker>(
        urdf_xml, srdf_xml, security_margin, this->get_logger(), this->get_clock(), &isTrackedLink);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load models: %s", e.what());
    throw;
  }

  // Initialize the full configuration vector to neutral — non-tracked joints
  // (base, casters, TMR drivetrain) will remain at this value permanently.
  Eigen::VectorXd q0 = collision_checker_->getNeutralConfiguration();
  current_joint_positions_.assign(q0.data(), q0.data() + q0.size());

  // Build joint_map_ only for tracked joints using idx_q as the direct index
  // into the full configuration vector. Multi-DOF joints (nq != 1, e.g.
  // continuous wheels/steering) are intentionally skipped and stay at neutral.
  joint_map_.clear();
  for (pinocchio::JointIndex i = 1;
       i < (pinocchio::JointIndex)collision_checker_->getModelNjoints(); ++i) {
    const std::string& name = collision_checker_->getModelJointName(i);
    int idx_q = collision_checker_->getModelJointIdxQ(i);
    int nq_j = collision_checker_->getModelJointNq(i);

    if (kTrackedJoints.count(name) == 0) {
      RCLCPP_INFO(this->get_logger(), "Skipping joint [%s] (idx_q=%d, nq=%d) — not in tracked set",
                  name.c_str(), idx_q, nq_j);
      continue;
    }
    if (nq_j != 1) {
      RCLCPP_WARN(this->get_logger(),
                  "Tracked joint [%s] has nq=%d (expected 1) — skipping to avoid corruption",
                  name.c_str(), nq_j);
      continue;
    }

    joint_map_[name] = static_cast<size_t>(idx_q);
    RCLCPP_INFO(this->get_logger(), "Tracking joint [%s] -> idx_q=%d", name.c_str(), idx_q);
  }

  RCLCPP_INFO(this->get_logger(), "model_.nq=%zu | tracked joints=%zu",
              collision_checker_->getModelNq(), joint_map_.size());

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->joint_state_callback(msg);
      });

  RCLCPP_INFO(this->get_logger(), "Self-Collision Monitor Active. (Margin: %.3f m)",
              security_margin);
}

void CollisionMonitorNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  // current_joint_positions_ has size model_.nq (full configuration vector).
  // Only tracked joints are updated here; all other entries remain at the
  // neutral configuration set during setup, keeping non-tracked joints
  // (base, casters, TMR drivetrain) safely at their neutral pose.
  for (size_t i = 0; i < msg->name.size(); ++i) {
    auto it = joint_map_.find(msg->name[i]);
    if (it != joint_map_.end() && i < msg->position.size()) {
      size_t idx = it->second;
      if (idx < current_joint_positions_.size()) {
        current_joint_positions_[idx] = msg->position[i];
      }
    }
  }

  bool collision = collision_checker_->checkCollision(current_joint_positions_, print_collisions_);
  auto collision_msg = std_msgs::msg::Bool();
  collision_msg.data = collision;
  collision_pub_->publish(collision_msg);

  if (collision) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "COLLISION DETECTED! Robot is in self-collision!");
  }
}

}  // namespace franka_selfcollision

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // No hardcoded params file — all parameters come from the launch file.
  rclcpp::NodeOptions options;
  auto node = std::make_shared<franka_selfcollision::CollisionMonitorNode>(options);

  auto param_client =
      std::make_shared<rclcpp::AsyncParametersClient>(node, "robot_state_publisher");
  param_client->wait_for_service();
  auto future = param_client->get_parameters({"robot_description"});
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto results = future.get();
    std::string robot_description = results[0].as_string();
    node->setup_collision_monitor(robot_description);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get robot_description parameter.");
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}