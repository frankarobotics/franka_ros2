#include "franka_robot_test.hpp"

TEST_F(FrankaRobotTests, whenInitializeTorqueInterfaceCalled_thenStartTorqueControlCalled) {
  EXPECT_CALL(*mock_libfranka_robot, startTorqueControl()).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeTorqueInterface();
}

TEST_F(FrankaRobotTests, whenInitializeJointVelocityInterfaceCalled_thenStartJointVelocityControl) {
  EXPECT_CALL(*mock_libfranka_robot, startJointVelocityControl(testing::_)).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeJointVelocityInterface();
}

TEST_F(FrankaRobotTests,
       whenInitializeJointtPositionInterfaceCalled_thenStartJointPositionControl) {
  EXPECT_CALL(*mock_libfranka_robot, startJointPositionControl(testing::_)).Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeJointPositionInterface();
}

TEST_F(FrankaRobotTests,
       whenInitializeCartesianVelocityInterfaceCalled_thenStartCartesianVelocityControl) {
  EXPECT_CALL(*mock_libfranka_robot,
              startCartesianVelocityControl(
                  research_interface::robot::Move::ControllerMode::kCartesianImpedance))
      .Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeCartesianVelocityInterface();
}

TEST_F(FrankaRobotTests, whenInitializeCartesianPoseInterfaceCalled_thenStartCartesianPoseControl) {
  EXPECT_CALL(*mock_libfranka_robot,
              startCartesianPoseControl(
                  research_interface::robot::Move::ControllerMode::kCartesianImpedance))
      .Times(1);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeCartesianPoseInterface();
}

TEST_F(FrankaRobotTests,
       givenCartesianVelocityControlIsStarted_whenReadOnceIsCalled_expectCorrectRobotState) {
  franka::RobotState robot_state;
  franka::Duration duration;
  robot_state.q_d = std::array<double, 7>{1, 2, 3, 1, 2, 3, 1};
  auto active_control_read_return_tuple = std::make_pair(robot_state, duration);

  EXPECT_CALL(*mock_active_control, readOnce())
      .WillOnce(testing::Return(active_control_read_return_tuple));
  EXPECT_CALL(*mock_libfranka_robot, startCartesianVelocityControl(testing::_))
      .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeCartesianVelocityInterface();
  auto actual_state = robot.readOnce();
  ASSERT_EQ(robot_state.q_d, actual_state.q_d);
}

TEST_F(FrankaRobotTests,
       givenCartesianPoseControlIsStarted_whenReadOnceIsCalled_expectCorrectRobotState) {
  franka::RobotState robot_state;
  franka::Duration duration;
  robot_state.q_d = std::array<double, 7>{1, 2, 3, 1, 2, 3, 1};
  auto active_control_read_return_tuple = std::make_pair(robot_state, duration);

  EXPECT_CALL(*mock_active_control, readOnce())
      .WillOnce(testing::Return(active_control_read_return_tuple));
  EXPECT_CALL(*mock_libfranka_robot, startCartesianPoseControl(testing::_))
      .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  robot.initializeCartesianPoseInterface();
  auto actual_state = robot.readOnce();
  ASSERT_EQ(robot_state.q_d, actual_state.q_d);
}

TEST_F(FrankaRobotTests,
       givenJointControlIsNotStarted_whenReadOnceIsCalled_expectCorrectRobotState) {
  franka::RobotState robot_state;
  robot_state.q_d = std::array<double, 7>{1, 2, 3, 1, 2, 3, 1};

  EXPECT_CALL(*mock_libfranka_robot, readOnce()).WillOnce(testing::Return(robot_state));

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));
  auto actual_state = robot.readOnce();

  ASSERT_EQ(robot_state.q_d, actual_state.q_d);
}

TEST_F(
    FrankaRobotTests,
    givenCartesianVelocityControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::vector<double>& cartesian_velocities{1, 2, 3, 1, 2, 3};
  std::array<double, 6> cartesian_velocities_array;
  std::copy(cartesian_velocities.begin(), cartesian_velocities.end(),
            cartesian_velocities_array.begin());
  const franka::CartesianVelocities expected_cartesian_velocities(cartesian_velocities_array);

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startCartesianVelocityControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testReadWriteOnce<void (franka_hardware::Robot::*)(), franka::CartesianVelocities,
                    std::vector<double>>(
      &franka_hardware::Robot::initializeCartesianVelocityInterface, expectCallFunction,
      cartesian_velocities, expected_cartesian_velocities);
}

TEST_F(FrankaRobotTests,
       givenCartesianPoseControlIsStart_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::vector<double> cartesian_pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 2, 3, 4, 1};
  std::array<double, 16> cartesian_pose_array{};
  std::copy(cartesian_pose.begin(), cartesian_pose.end(), cartesian_pose_array.begin());
  const franka::CartesianPose expected_cartesian_pose(cartesian_pose_array);

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startCartesianPoseControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testReadWriteOnce<void (franka_hardware::Robot::*)(), franka::CartesianPose, std::vector<double>>(
      &franka_hardware::Robot::initializeCartesianPoseInterface, expectCallFunction, cartesian_pose,
      expected_cartesian_pose);
}

TEST_F(
    FrankaRobotTests,
    givenJointPositionControlIsControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::vector<double> joint_positions{1, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> joint_positions_array{};
  std::copy(joint_positions.begin(), joint_positions.end(), joint_positions_array.begin());
  const franka::JointPositions expected_joint_positions(joint_positions_array);

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startJointPositionControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testReadWriteOnce<void (franka_hardware::Robot::*)(), franka::JointPositions,
                    std::vector<double>>(&franka_hardware::Robot::initializeJointPositionInterface,
                                         expectCallFunction, joint_positions,
                                         expected_joint_positions);
}

TEST_F(
    FrankaRobotTests,
    givenJointVelocityControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::vector<double> joint_velocities{1, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> joint_velocities_array{};
  std::copy(joint_velocities.begin(), joint_velocities.end(), joint_velocities_array.begin());
  const franka::JointVelocities expected_joint_velocities(joint_velocities_array);

  const auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startJointVelocityControl(testing::_))
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testReadWriteOnce<void (franka_hardware::Robot::*)(), franka::JointVelocities,
                    std::vector<double>>(&franka_hardware::Robot::initializeJointVelocityInterface,
                                         expectCallFunction, joint_velocities,
                                         expected_joint_velocities);
}

TEST_F(FrankaRobotTests,
       givenEffortControlIsStarted_whenWriteOnceIsCalled_expectActiveControlWriteOnceCalled) {
  const std::vector<double>& joint_torques{1, 0, 0, 0, 0, 0, 0};
  // Torque rate limiter defaulted to active
  const franka::Torques expected_joint_torques{0.999999, 0, 0, 0, 0, 0, 0};

  auto expectCallFunction = [this]() {
    EXPECT_CALL(*mock_libfranka_robot, startTorqueControl())
        .WillOnce(testing::Return(testing::ByMove((std::move(mock_active_control)))));
  };

  testReadWriteOnce<void (franka_hardware::Robot::*)(), franka::Torques, std::vector<double>>(
      &franka_hardware::Robot::initializeTorqueInterface, expectCallFunction, joint_torques,
      expected_joint_torques);
}

TEST_F(FrankaRobotTests,
       givenControlIsNotStarted_whenWriteOnceIsCalled_expectRuntimeExceptionToBeThrown) {
  const std::array<double, 7> joint_torques{1, 0, 0, 0, 0, 0, 0};
  const std::vector<double> joint_torques_vector{joint_torques.cbegin(), joint_torques.cend()};
  const franka::Torques joint_torques_franka(joint_torques);

  const std::array<double, 6> cartesian_velocities{1, 0, 0, 0, 0, 0};
  const std::vector<double> cartesian_velocities_vector{cartesian_velocities.cbegin(),
                                                        cartesian_velocities.cend()};
  const franka::CartesianVelocities cartesian_franka_velocities(cartesian_velocities);

  const std::array<double, 16> cartesian_pose{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  const std::vector<double> cartesian_pose_vector{cartesian_pose.cbegin(), cartesian_pose.cend()};
  const franka::CartesianPose expected_cartesian_pose(cartesian_pose);

  const std::array<double, 2> elbow{0.0, 0.0};
  const std::vector<double> elbow_vector{elbow.cbegin(), elbow.cend()};
  const franka::CartesianPose expected_cartesian_pose_with_elbow(cartesian_pose, elbow);
  const franka::CartesianVelocities expected_cartesian_velocities_with_elbow(cartesian_velocities,
                                                                             elbow);

  EXPECT_CALL(*mock_active_control, writeOnce(joint_torques_franka)).Times(0);
  EXPECT_CALL(*mock_active_control, writeOnce(cartesian_franka_velocities)).Times(0);
  EXPECT_CALL(*mock_active_control, writeOnce(expected_cartesian_pose)).Times(0);
  EXPECT_CALL(*mock_active_control, writeOnce(expected_cartesian_velocities_with_elbow)).Times(0);
  EXPECT_CALL(*mock_active_control, writeOnce(expected_cartesian_pose_with_elbow)).Times(0);

  franka_hardware::Robot robot(std::move(mock_libfranka_robot), std::move(mock_model));

  EXPECT_THROW(robot.writeOnce(joint_torques_vector), std::runtime_error);
  EXPECT_THROW(robot.writeOnce(cartesian_velocities_vector), std::runtime_error);
  EXPECT_THROW(robot.writeOnce(cartesian_velocities_vector, elbow_vector), std::runtime_error);
  EXPECT_THROW(robot.writeOnce(cartesian_pose_vector), std::runtime_error);
  EXPECT_THROW(robot.writeOnce(cartesian_pose_vector, elbow_vector), std::runtime_error);
}
