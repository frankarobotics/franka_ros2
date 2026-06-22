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

#include "franka_example_controllers/fr3/cartesian_impedance_example_controller.hpp"

#include <cassert>

namespace franka_example_controllers {

CartesianImpedanceExampleController::CartesianGains
CartesianImpedanceExampleController::buildGains(const std::array<double, num_cartesian_dof>& k) {
  CartesianGains g;
  for (int i = 0; i < num_cartesian_dof; ++i) {
    const double ki = std::max(0.0, k[i]);
    g.stiffness(i, i) = ki;
    g.damping(i, i) = 2.0 * std::sqrt(ki);  // critically-damped
  }
  return g;
}

CartesianImpedanceExampleController::CallbackReturn
CartesianImpedanceExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<double>("nullspace_stiffness", 20.0);
    auto_declare<double>("translational_stiffness", 150.0);
    auto_declare<double>("rotational_stiffness", 10.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; i++) {
    command_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // 1) Joint position/velocity — read directly from hardware_interface state interfaces.
  //    Order is (j1.pos, j1.vel, j2.pos, j2.vel, ...) so readJointState() can index by 2*i.
  for (int i = 1; i <= num_joints; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  // 2) Cartesian pose state via FrankaCartesianPoseInterface (16 column-major matrix entries).
  for (const auto& name : franka_cartesian_pose_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(name);
  }

  // 3) Robot model + state for mass, coriolis, jacobian.
  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(name);
  }

  return state_interfaces_config;
}

CartesianImpedanceExampleController::CallbackReturn
CartesianImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  // Match the franka_hardware export pattern: prefix + index + "/cartesian_pose_state".
  const std::string cartesian_pose_prefix =
      arm_prefix_.empty() ? std::string{} : arm_prefix_ + "_";

  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          cartesian_pose_prefix, k_elbow_activated_);

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/" + k_robot_model_interface_name,
      arm_id_ + "/" + k_robot_state_interface_name);

  const double t_k = get_node()->get_parameter("translational_stiffness").as_double();
  const double r_k = get_node()->get_parameter("rotational_stiffness").as_double();
  const double n_k = get_node()->get_parameter("nullspace_stiffness").as_double();

  CartesianGains gains = buildGains({t_k, t_k, t_k, r_k, r_k, r_k});
  cartesian_stiffness_ = gains.stiffness;
  cartesian_damping_ = gains.damping;
  nullspace_stiffness_ = n_k;
  cartesian_gains_buffer_.initRT(gains);
  nullspace_stiffness_buffer_.initRT(n_k);

  target_pose_buffer_.initRT(TargetPose{});

  sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/equilibrium_pose", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianImpedanceExampleController::equilibriumPoseCallback, this,
                std::placeholders::_1));

  srv_set_cartesian_stiffness_ =
      get_node()->create_service<franka_msgs::srv::SetCartesianStiffness>(
          "~/set_cartesian_stiffness",
          std::bind(&CartesianImpedanceExampleController::setCartesianStiffnessCallback, this,
                    std::placeholders::_1, std::placeholders::_2));

  param_cb_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&CartesianImpedanceExampleController::onParameterUpdate, this,
                std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CartesianImpedanceExampleController::CallbackReturn
CartesianImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  readJointState();
  Eigen::Quaterniond orientation_init;
  Eigen::Vector3d position_init;
  std::tie(orientation_init, position_init) =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  orientation_init.normalize();

  position_d_ = position_init;
  initial_position_ = position_init;
  elapsed_time_ = 0.0;
  orientation_d_ = orientation_init;
  q_d_nullspace_ = q_;

  TargetPose initial_target;
  initial_target.position = position_init;
  initial_target.orientation = orientation_init;
  target_pose_buffer_.initRT(initial_target);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceExampleController::readJointState() {
  // The first 2 * num_joints entries are joint position/velocity, in the order declared
  // by state_interface_configuration().
  for (int i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_optional().value();
    dq_(i) = velocity_interface.get_optional().value();
  }
}

controller_interface::return_type CartesianImpedanceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  readJointState();

  Eigen::Quaterniond orientation;
  Eigen::Vector3d position;
  std::tie(orientation, position) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();

  std::array<double, num_joints> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, num_cartesian_dof * num_joints> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, num_joints, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, num_cartesian_dof, num_joints>> jacobian(jacobian_array.data());

  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = position;
  transform.rotate(orientation.toRotationMatrix());

  const TargetPose target = *target_pose_buffer_.readFromRT();
  updateMotionTarget(period.seconds(), target.orientation);

  const CartesianGains target_gains = *cartesian_gains_buffer_.readFromRT();
  const double target_nullspace_stiffness = *nullspace_stiffness_buffer_.readFromRT();

  const auto error = computeError(position, orientation, transform);

  Eigen::Matrix<double, num_joints, 1> tau_task, tau_nullspace, tau_d;

  // Damped pseudo-inverse of jacobian transpose
  Eigen::MatrixXd jacobian_transpose_pinv;
  double lambda = 0.2;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian.transpose(),
      Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();
  Eigen::MatrixXd S = jacobian.transpose();
  S.setZero();
  for (int i = 0; i < sing_vals.size(); i++) {
    S(i, i) = sing_vals(i) / (sing_vals(i) * sing_vals(i) + lambda * lambda);
  }
  jacobian_transpose_pinv = svd.matrixV() * S.transpose() * svd.matrixU().transpose();

  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq_));

  tau_nullspace << (Eigen::Matrix<double, num_joints, num_joints>::Identity() -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q_) -
                        2.0 * std::sqrt(nullspace_stiffness_) * dq_);

  tau_d << tau_task + tau_nullspace + coriolis;

  for (int i = 0; i < num_joints; i++) {
    if (!command_interfaces_[i].set_value(tau_d[i])) {
      return controller_interface::return_type::ERROR;
    }
  }

  cartesian_stiffness_ =
      filter_params_ * target_gains.stiffness + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * target_gains.damping + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * target_nullspace_stiffness + (1.0 - filter_params_) * nullspace_stiffness_;

  position_d_ = filter_params_ * target.position + (1.0 - filter_params_) * position_d_;
  // Keep the working orientation on the same hemisphere as the target before slerp.
  if (orientation_d_.coeffs().dot(target.orientation.coeffs()) < 0.0) {
    orientation_d_.coeffs() = -orientation_d_.coeffs();
  }
  orientation_d_ = orientation_d_.slerp(filter_params_, target.orientation);
  orientation_d_.normalize();

  return controller_interface::return_type::OK;
}

void CartesianImpedanceExampleController::updateMotionTarget(
    double period_seconds, const Eigen::Quaterniond& orientation) {
  elapsed_time_ += period_seconds;
  double angle = M_PI / 4.0 * (1 - std::cos(M_PI / 5.0 * elapsed_time_));
  double delta_x = 0.1 * std::sin(angle);
  double delta_z = 0.1 * (std::cos(angle) - 1);
  TargetPose motion_target;
  motion_target.position = initial_position_ + Eigen::Vector3d(delta_x, 0.0, delta_z);
  motion_target.orientation = orientation;
  target_pose_buffer_.writeFromNonRT(motion_target);
}

Eigen::Matrix<double, 6, 1>
CartesianImpedanceExampleController::computeError(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const Eigen::Affine3d& transform) const {
  Eigen::Matrix<double, num_cartesian_dof, 1> error;
  error.head(3) << position - position_d_;
  Eigen::Quaterniond orientation_corrected = orientation;
  if (orientation_d_.coeffs().dot(orientation_corrected.coeffs()) < 0.0) {
    orientation_corrected.coeffs() = -orientation_corrected.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation_corrected.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error.tail(3) << -transform.rotation() * error.tail(3);
  return error;
}

void CartesianImpedanceExampleController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  TargetPose target;
  target.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  target.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                          msg->pose.orientation.y, msg->pose.orientation.z);

  // Reject malformed input rather than letting NaNs poison the RT thread.
  if (target.orientation.coeffs().norm() < 1e-6) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Discarding equilibrium pose with degenerate quaternion.");
    return;
  }
  target.orientation.normalize();

  target_pose_buffer_.writeFromNonRT(target);
}

void CartesianImpedanceExampleController::setCartesianStiffnessCallback(
    const std::shared_ptr<franka_msgs::srv::SetCartesianStiffness::Request> request,
    std::shared_ptr<franka_msgs::srv::SetCartesianStiffness::Response> response) {
  std::array<double, num_cartesian_dof> k{};
  bool valid = true;
  for (size_t i = 0; i < num_cartesian_dof; ++i) {
    k[i] = request->cartesian_stiffness[i];
    if (!std::isfinite(k[i]) || k[i] < 0.0) {
      valid = false;
      break;
    }
  }
  if (!valid) {
    response->success = false;
    response->error = "Cartesian stiffness must contain 6 finite, non-negative values.";
    return;
  }

  cartesian_gains_buffer_.writeFromNonRT(buildGains(k));
  response->success = true;
  response->error = "";
}

rcl_interfaces::msg::SetParametersResult
CartesianImpedanceExampleController::onParameterUpdate(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& p : parameters) {
    if (p.get_name() == "nullspace_stiffness") {
      const double v = p.as_double();
      if (!std::isfinite(v) || v < 0.0) {
        result.successful = false;
        result.reason = "nullspace_stiffness must be finite and non-negative";
        return result;
      }
      nullspace_stiffness_buffer_.writeFromNonRT(v);
    }
  }
  return result;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)