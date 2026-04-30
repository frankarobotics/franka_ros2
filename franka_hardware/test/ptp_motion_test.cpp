#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <optional>
#include <stdexcept>

#include "franka_hardware/actions_helper/ptp_motion_handler.hpp"
#include "franka_hardware_mocks/franka_active_control_mock.hpp"
#include "franka_hardware_mocks/franka_hardware_robot_mock.hpp"
#include "franka_hardware_mocks/franka_robot_mock.hpp"

constexpr size_t kMaxCounter = 1000;

namespace franka {

static constexpr double k_EPS = 1e-5;

template <typename T>
bool compareWithTolerance(const T& lhs, const T& rhs) {
  return std::equal(lhs.cbegin(), lhs.cend(), rhs.cbegin(),
                    [](double lhs_element, double rhs_element) {
                      return std::abs(lhs_element - rhs_element) < k_EPS;
                    });
}

bool operator==(const franka::JointPositions& lhs, const franka::JointPositions& rhs) {
  auto is_motion_finished_equal = lhs.motion_finished == rhs.motion_finished;
  return is_motion_finished_equal && compareWithTolerance(lhs.q, rhs.q);
}

std::ostream& operator<<(std::ostream& os, const JointPositions& jp) {
  os << "JointPositions{q=[";
  for (size_t i = 0; i < jp.q.size(); ++i) {
    if (i)
      os << ", ";
    os << std::fixed << std::setprecision(6) << jp.q[i];
  }
  os << "], motion_finished=" << std::boolalpha << jp.motion_finished << "}";
  return os;
}

}  // namespace franka

MATCHER_P(JointPositionsEq, expected, "") {
  return arg == expected;
}

class PTPMotionTests : public ::testing::Test {
 protected:
  void SetUp() override {
    mock_robot = std::make_shared<MockRobot>();
    mock_libfranka_robot = std::make_shared<MockFrankaRobot>();
    mock_active_control = std::make_unique<MockActiveControl>();
    ptp_motion_handler = std::make_unique<franka_hardware::PTPMotionHandler>(mock_robot);

    std::copy(std::cbegin(default_goal_joint_configuration),
              std::cend(default_goal_joint_configuration), std::begin(default_robot_state.q));
    std::copy(std::cbegin(default_zero_joint_velocity), std::cend(default_zero_joint_velocity),
              std::begin(default_robot_state.dq));

    std::copy(std::cbegin(default_goal_joint_configuration),
              std::cend(default_goal_joint_configuration), std::begin(default_joint_positions.q));
    default_joint_positions.motion_finished = false;
    stopping_joint_positions.motion_finished = true;
  }

  std::shared_ptr<MockRobot> mock_robot;
  std::shared_ptr<MockFrankaRobot> mock_libfranka_robot;
  std::unique_ptr<MockActiveControl> mock_active_control;
  std::unique_ptr<franka_hardware::PTPMotionHandler> ptp_motion_handler;

  std::vector<double> default_goal_joint_configuration{0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.5};
  std::vector<double> not_goal_joint_configuration{0.5, -0.5, 0.5, -1.5, 0.5, 1.0, 0.0};
  std::vector<double> default_zero_joint_velocity{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  franka::RobotState default_robot_state{};
  // This value will be overwritten in the SetUp to the default goal joint configuration
  franka::JointPositions default_joint_positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  franka::JointPositions stopping_joint_positions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  auto startNewMotion() -> std::string {
    std::vector<double> maximum_joint_velocities(7, 1.0);
    franka_msgs::action::PTPMotion::Goal goal;
    goal.goal_joint_configuration = default_goal_joint_configuration;
    goal.maximum_joint_velocities = maximum_joint_velocities;
    goal.goal_tolerance = 0.01;

    auto robot_state = franka::RobotState{};
    auto expected_optional = std::optional<std::vector<double>>(maximum_joint_velocities);
    EXPECT_CALL(*mock_robot, getCurrentState())
        .WillRepeatedly(::testing::Invoke([&robot_state]() { return robot_state; }));
    EXPECT_CALL(*mock_libfranka_robot,
                startAsyncJointPositionControl(::testing::_, ::testing::Eq(expected_optional)))
        .WillOnce(::testing::Return(::testing::ByMove(std::move(mock_active_control))));

    auto command_result = ptp_motion_handler->startNewPTPMotion(
        mock_libfranka_robot, std::make_shared<franka_msgs::action::PTPMotion::Goal>(goal));

    EXPECT_FALSE(command_result.motion_id.empty());
    EXPECT_EQ(command_result.result->target_status.status, franka_msgs::msg::TargetStatus::IDLE);

    return command_result.motion_id;
  }

  auto runUntilExecuting(const std::string& current_motion_id) -> void {
    default_robot_state.robot_mode = franka::RobotMode::kMove;
    std::copy(std::begin(not_goal_joint_configuration), std::end(not_goal_joint_configuration),
              std::begin(default_robot_state.q_d));
    EXPECT_CALL(*mock_robot, getCurrentState())
        .WillRepeatedly(::testing::Invoke([this]() { return default_robot_state; }));

    size_t counter = 0;
    while (counter < kMaxCounter) {
      auto target_feedback = ptp_motion_handler->getFeedback(current_motion_id);
      if (target_feedback.status == franka::TargetStatus::kExecuting) {
        return;
      }
      ++counter;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    FAIL() << "Timeout waiting for motion to start moving.";
  }

  auto runUntilTargetReached(const std::string& current_motion_id) -> void {
    default_robot_state.robot_mode = franka::RobotMode::kMove;
    std::copy(std::begin(default_goal_joint_configuration),
              std::end(default_goal_joint_configuration), std::begin(default_robot_state.q_d));
    EXPECT_CALL(*mock_robot, getCurrentState())
        .WillRepeatedly(::testing::Invoke([this]() { return default_robot_state; }));

    size_t counter = 0;
    while (counter < kMaxCounter) {
      auto target_feedback = ptp_motion_handler->getFeedback(current_motion_id);
      if (target_feedback.status == franka::TargetStatus::kTargetReached) {
        return;
      }
      ++counter;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    FAIL() << "Timeout waiting for motion to reach target.";
  }
};

TEST_F(PTPMotionTests, givenValidGoal_whenStartNewPTPMotion_thenMotionStartsSuccessfully) {
  std::vector<double> maximum_joint_velocities(7, 1.0);
  franka_msgs::action::PTPMotion::Goal goal;
  goal.goal_joint_configuration = {0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.5};
  goal.maximum_joint_velocities = maximum_joint_velocities;
  goal.goal_tolerance = 0.01;

  auto robot_state = franka::RobotState{};
  EXPECT_CALL(*mock_robot, getCurrentState())
      .WillRepeatedly(::testing::Invoke([&robot_state]() { return robot_state; }));
  auto expected_optional = std::optional<std::vector<double>>(maximum_joint_velocities);
  EXPECT_CALL(*mock_libfranka_robot,
              startAsyncJointPositionControl(::testing::_, ::testing::Eq(expected_optional)))
      .WillOnce(::testing::Return(::testing::ByMove(std::move(mock_active_control))));

  auto command_result = ptp_motion_handler->startNewPTPMotion(
      mock_libfranka_robot, std::make_shared<franka_msgs::action::PTPMotion::Goal>(goal));

  ASSERT_FALSE(command_result.motion_id.empty());
  ASSERT_EQ(command_result.result->target_status.status, franka_msgs::msg::TargetStatus::IDLE);
}

TEST_F(PTPMotionTests,
       givenUninitializedPositionControlHandler_whenStartNewPTPMotion_thenAbortIsReturned) {
  franka_msgs::action::PTPMotion::Goal goal;
  goal.goal_joint_configuration = {0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.5};
  goal.maximum_joint_velocities = std::vector<double>(7, 1.0);
  goal.goal_tolerance = 0.01;

  EXPECT_CALL(*mock_libfranka_robot, startAsyncJointPositionControl(::testing::_, ::testing::_))
      .WillOnce(
          ::testing::Throw(franka::ControlException("startAsyncJointPositionControl failed")));

  auto command_result = ptp_motion_handler->startNewPTPMotion(
      mock_libfranka_robot, std::make_shared<franka_msgs::action::PTPMotion::Goal>(goal));

  ASSERT_TRUE(command_result.motion_id.empty());
  ASSERT_EQ(command_result.result->target_status.status, franka_msgs::msg::TargetStatus::ABORTED);
  ASSERT_FALSE(command_result.result->error_message.empty());
}

TEST_F(PTPMotionTests, givenNoMotionYet_whenGetFeedback_thenNothingActive) {
  auto non_existing_motion_id = "non_existent_motion_id";
  auto target_feedback = ptp_motion_handler->getFeedback(non_existing_motion_id);

  ASSERT_EQ(target_feedback.status, franka::TargetStatus::kAborted);
  ASSERT_TRUE(target_feedback.error_message.has_value());
}

TEST_F(PTPMotionTests, givenValidMotion_whenStartNewPTPMotion_thenCanGetFeedback) {
  auto new_motion_id = startNewMotion();

  auto target_feedback = ptp_motion_handler->getFeedback(new_motion_id);
  ASSERT_EQ(target_feedback.status, franka::TargetStatus::kIdle);
  ASSERT_FALSE(target_feedback.error_message.has_value());
}

TEST_F(PTPMotionTests, givenValidMotion_whenReachingTarget_thenFeedbackShowsTargetReached) {
  auto new_motion_id = startNewMotion();

  default_robot_state.robot_mode = franka::RobotMode::kMove;
  EXPECT_CALL(*mock_robot, getCurrentState())
      .WillRepeatedly(::testing::Invoke([this]() { return default_robot_state; }));

  runUntilTargetReached(new_motion_id);
}

TEST_F(PTPMotionTests, givenValidMotion_whenCancelMotion_thenMotionIsCancelled) {
  default_joint_positions.motion_finished = false;
  auto active_control_raw = mock_active_control.get();
  EXPECT_CALL(*active_control_raw, writeOnce(::testing::An<const franka::JointPositions&>()))
      .Times(::testing::AtLeast(1));

  default_robot_state.robot_mode = franka::RobotMode::kMove;
  EXPECT_CALL(*mock_robot, getCurrentState())
      .WillRepeatedly(::testing::Invoke([this]() { return default_robot_state; }));

  auto new_motion_id = startNewMotion();
  runUntilExecuting(new_motion_id);

  ptp_motion_handler->cancelMotion();

  auto feedback = ptp_motion_handler->getFeedback(new_motion_id);
  ASSERT_NE(feedback.status, franka::TargetStatus::kExecuting);
}
