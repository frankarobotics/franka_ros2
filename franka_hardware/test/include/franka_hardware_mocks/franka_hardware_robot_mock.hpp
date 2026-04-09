#pragma once

#include <gmock/gmock.h>

#include <franka_hardware/robot.hpp>

#include "franka_model_mock.hpp"

class MockRobot : public franka_hardware::Robot {
 public:
  MOCK_METHOD(void, initializeJointPositionInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianVelocityInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianPoseInterface, (), (override));
  MOCK_METHOD(void, initializeTorqueInterface, (), (override));
  MOCK_METHOD(void, initializeJointVelocityInterface, (), (override));
  MOCK_METHOD(void, stopRobot, (), (override));
  MOCK_METHOD(franka::RobotState, readOnce, (), (override));
  MOCK_METHOD(MockModel*, getModel, (), (override));
  MOCK_METHOD(void, writeOnce, ((const std::vector<double>&)), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::vector<double>&), (const std::vector<double>&)),
              (override));
  MOCK_METHOD(void,
              setJointStiffness,
              (const franka_msgs::srv::SetJointStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setCartesianStiffness,
              (const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, setLoad, (const franka_msgs::srv::SetLoad::Request::SharedPtr&), (override));
  MOCK_METHOD(void,
              setTCPFrame,
              (const franka_msgs::srv::SetTCPFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setStiffnessFrame,
              (const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setForceTorqueCollisionBehavior,
              (const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setFullCollisionBehavior,
              (const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, automaticErrorRecovery, (), (override));
  MOCK_METHOD(std::shared_ptr<franka::Robot>, getRobot, (), (override));
  MOCK_METHOD(const franka::RobotState&, getCurrentState, (), (override));
};
