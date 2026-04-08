#include "franka_hardware_cartesian_command_interface_test.hpp"

using ::testing::_;

const std::vector<double> kZeroCartesianVelocities{0, 0, 0, 0, 0, 0};
const std::vector<double> kZeroCartesianPose{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    given_correct_number_of_start_cartesian_command_interface_when_prepare_command_mode_interface_is_called_expect_success) {
  auto [command_interfaces, command_interface_name] = GetParam();

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    start_interface.push_back(name + "/" + command_interface_name);
  }

  std::vector<std::string> stop_interface = {};
  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  start_interface.clear();
  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    stop_interface.push_back(name + "/" + command_interface_name);
  }

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    given_correct_number_of_start_cartesian_command_and_elbow_command_interface_when_prepare_command_mode_interface_is_called_expect_success) {
  auto [command_interfaces, command_interface_name] = GetParam();

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    start_interface.push_back(name + "/" + command_interface_name);
  }

  std::vector<std::string> stop_interface = {};
  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  start_interface.clear();

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    stop_interface.push_back(name + "/" + command_interface_name);
  }
  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_start_interface_number_expect_throw) {
  auto [command_interfaces, command_interface_name] = GetParam();

  std::vector<std::string> start_interface, stop_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    stop_interface.push_back(name + "/" + command_interface_name);
  }

  start_interface = {"fr3_joint1/" + k_position_controller};

  EXPECT_THROW(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                             stop_interface),
               std::invalid_argument);
}

TEST_P(
    FrankaCartesianCommandInterfaceTest,
    when_prepare_command_mode_interface_is_called_with_invalid_stop_interface_number_expect_throw) {
  auto [command_interfaces, command_interface_name] = GetParam();

  std::vector<std::string> start_interface, stop_interface;

  for (size_t i = 0; i < command_interfaces.size(); i++) {
    const std::string name = command_interfaces[i];
    start_interface.push_back(name + "/" + command_interface_name);
  }

  stop_interface = {"fr3_joint1/" + k_position_controller};

  EXPECT_THROW(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                             stop_interface),
               std::invalid_argument);
}

TEST_F(FrankaCartesianCommandInterfaceTest,
       given_read_is_not_called_when_write_is_called_expect_robot_writeOnce_is_not_called) {
  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianVelocityInterface());

  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianVelocities, _)).Times(0);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(FrankaCartesianCommandInterfaceTest,
       given_cartesian_velocity_and_elbow_set_when_read_and_write_called_expect_success) {
  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianVelocityInterface());

  EXPECT_CALL(*default_mock_robot, readOnce()).Times(1);
  EXPECT_CALL(*default_mock_robot, writeOnce(_)).Times(0);
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianVelocities, _)).Times(1);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_cartesian_velocity_and_elbow_set_and_elbow_has_infinite_values_when_write_called_expect_error) {
  franka::RobotState robot_state;
  robot_state.elbow = std::array<double, 2>{std::numeric_limits<double>::infinity()};

  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianVelocityInterface());

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianVelocities)).Times(0);
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianVelocities, _)).Times(0);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::ERROR);
}

TEST_F(FrankaCartesianCommandInterfaceTest,
       given_cartesian_velocity_is_claimed_when_perform_mode_switch_is_called_expect_success) {
  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianVelocityInterface());

  // Only cartesian velocity command
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianVelocities));

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_velocities_names.size(); i++) {
    const std::string name = k_hw_cartesian_velocities_names[i];
    start_interface.push_back(name + "/" + k_cartesian_velocity_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_only_elbow_command_is_claimed_without_cartesian_velocity_when_perform_mode_switch_is_called_expect_error) {
  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::ERROR);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_cartesian_pose_interface_is_ready_when_write_called_without_read_robot_write_once_will_not_be_called) {
  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianPoseInterface());

  EXPECT_CALL(*default_mock_robot, readOnce()).Times(0);
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianPose)).Times(0);
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianPose, _)).Times(0);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_pose_names.size(); i++) {
    const std::string name = k_hw_cartesian_pose_names[i];
    start_interface.push_back(name + "/" + k_cartesian_pose_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_cartesian_pose_interface_is_ready_when_write_called_with_read_robot_write_once_will_be_called) {
  franka::RobotState robot_state;
  robot_state.O_T_EE = std::array<double, 16>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                              0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  auto expected_first_cartesian_pose_command =
      std::vector<double>{robot_state.O_T_EE.cbegin(), robot_state.O_T_EE.cend()};

  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianPoseInterface());

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, writeOnce(expected_first_cartesian_pose_command)).Times(1);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_pose_names.size(); i++) {
    const std::string name = k_hw_cartesian_pose_names[i];
    start_interface.push_back(name + "/" + k_cartesian_pose_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_cartesian_pose_and_elbow_set_and_elbow_has_infinite_values_when_write_called_expect_error) {
  franka::RobotState robot_state;
  robot_state.elbow = std::array<double, 2>{std::numeric_limits<double>::infinity()};

  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianPoseInterface());

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, writeOnce(_)).Times(0);
  EXPECT_CALL(*default_mock_robot, writeOnce(kZeroCartesianPose, _)).Times(0);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_pose_names.size(); i++) {
    const std::string name = k_hw_cartesian_pose_names[i];
    start_interface.push_back(name + "/" + k_cartesian_pose_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  // can call write only after performing command mode switch
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::ERROR);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    given_cartesian_pose_interface_is_ready_with_elbow_when_read_and_write_called_expect_correct_elbow_and_cartesian_pose) {
  franka::RobotState robot_state;
  robot_state.O_T_EE = std::array<double, 16>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0,
                                              0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  robot_state.elbow = std::array<double, 2>{0.0, 2.0};

  auto expected_first_cartesian_pose_command =
      std::vector<double>{robot_state.O_T_EE.cbegin(), robot_state.O_T_EE.cend()};
  auto expected_first_elbow_command =
      std::vector<double>{robot_state.elbow.cbegin(), robot_state.elbow.cend()};

  EXPECT_CALL(*default_mock_robot, stopRobot());
  EXPECT_CALL(*default_mock_robot, initializeCartesianPoseInterface());

  EXPECT_CALL(*default_mock_robot, readOnce()).WillOnce(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot,
              writeOnce(expected_first_cartesian_pose_command, expected_first_elbow_command))
      .Times(1);

  std::vector<std::string> start_interface;

  for (size_t i = 0; i < k_hw_cartesian_pose_names.size(); i++) {
    const std::string name = k_hw_cartesian_pose_names[i];
    start_interface.push_back(name + "/" + k_cartesian_pose_command_interface_name);
  }

  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    const std::string name = k_hw_elbow_command_names[i];
    start_interface.push_back(name + "/" + k_elbow_command_interface_name);
  }

  std::vector<std::string> stop_interface = {};

  ASSERT_EQ(default_franka_hardware_interface.prepare_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.perform_command_mode_switch(start_interface,
                                                                          stop_interface),
            hardware_interface::return_type::OK);

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(default_franka_hardware_interface.read(time, duration),
            hardware_interface::return_type::OK);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    givenCartesianPoseActiveAndUsed_whenOnActivateCalledAgain_expectWriteBlocksUntilRead) {
  franka::RobotState robot_state;
  robot_state.O_T_EE = std::array<double, 16>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.5, 1.0};
  auto expected_pose =
      std::vector<double>{robot_state.O_T_EE.cbegin(), robot_state.O_T_EE.cend()};

  MockModel mock_model;
  MockModel* model_address = &mock_model;

  EXPECT_CALL(*default_mock_robot, stopRobot()).Times(testing::AnyNumber());
  EXPECT_CALL(*default_mock_robot, readOnce()).WillRepeatedly(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, getModel()).WillRepeatedly(testing::Return(model_address));
  EXPECT_CALL(*default_mock_robot, initializeCartesianPoseInterface()).Times(2);

  std::vector<std::string> start_interface;
  for (size_t i = 0; i < k_hw_cartesian_pose_names.size(); i++) {
    start_interface.push_back(k_hw_cartesian_pose_names[i] + "/" +
                              k_cartesian_pose_command_interface_name);
  }
  std::vector<std::string> stop_interface = {};

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  // Phase 1: normal cycle
  default_franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface);
  default_franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface);
  default_franka_hardware_interface.read(time, duration);
  default_franka_hardware_interface.write(time, duration);

  // Phase 2: re-activate
  default_franka_hardware_interface.on_activate(rclcpp_lifecycle::State());
  default_franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface);
  default_franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface);

  // Write before read — should NOT send command
  EXPECT_CALL(*default_mock_robot, writeOnce(expected_pose)).Times(0);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);

  // After read — should send current O_T_EE
  EXPECT_CALL(*default_mock_robot, writeOnce(expected_pose)).Times(1);
  default_franka_hardware_interface.read(time, duration);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

TEST_F(
    FrankaCartesianCommandInterfaceTest,
    givenCartesianPoseWithElbowActiveAndUsed_whenOnActivateCalledAgain_expectWriteBlocksUntilRead) {
  franka::RobotState robot_state;
  robot_state.O_T_EE = std::array<double, 16>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.5, 1.0};
  robot_state.elbow = std::array<double, 2>{0.3, 1.0};

  auto expected_pose =
      std::vector<double>{robot_state.O_T_EE.cbegin(), robot_state.O_T_EE.cend()};
  auto expected_elbow =
      std::vector<double>{robot_state.elbow.cbegin(), robot_state.elbow.cend()};

  MockModel mock_model;
  MockModel* model_address = &mock_model;

  EXPECT_CALL(*default_mock_robot, stopRobot()).Times(testing::AnyNumber());
  EXPECT_CALL(*default_mock_robot, readOnce()).WillRepeatedly(testing::Return(robot_state));
  EXPECT_CALL(*default_mock_robot, getModel()).WillRepeatedly(testing::Return(model_address));
  EXPECT_CALL(*default_mock_robot, initializeCartesianPoseInterface()).Times(2);

  std::vector<std::string> start_interface;
  for (size_t i = 0; i < k_hw_cartesian_pose_names.size(); i++) {
    start_interface.push_back(k_hw_cartesian_pose_names[i] + "/" +
                              k_cartesian_pose_command_interface_name);
  }
  for (size_t i = 0; i < k_hw_elbow_command_names.size(); i++) {
    start_interface.push_back(k_hw_elbow_command_names[i] + "/" +
                              k_elbow_command_interface_name);
  }
  std::vector<std::string> stop_interface = {};

  const auto time = rclcpp::Time(0, 0);
  const auto duration = rclcpp::Duration(0, 0);

  // Phase 1: normal cycle
  default_franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface);
  default_franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface);
  default_franka_hardware_interface.read(time, duration);
  default_franka_hardware_interface.write(time, duration);

  // Phase 2: re-activate
  default_franka_hardware_interface.on_activate(rclcpp_lifecycle::State());
  default_franka_hardware_interface.prepare_command_mode_switch(start_interface, stop_interface);
  default_franka_hardware_interface.perform_command_mode_switch(start_interface, stop_interface);

  // Write before read — should NOT send command
  EXPECT_CALL(*default_mock_robot, writeOnce(expected_pose, expected_elbow)).Times(0);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);

  // After read — should send current pose + elbow
  EXPECT_CALL(*default_mock_robot, writeOnce(expected_pose, expected_elbow)).Times(1);
  default_franka_hardware_interface.read(time, duration);
  ASSERT_EQ(default_franka_hardware_interface.write(time, duration),
            hardware_interface::return_type::OK);
}

int main(int argc, char** argv) {
  rclcpp::init(0, nullptr);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
