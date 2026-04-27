#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/component_parser.hpp>

#include <franka_hardware_mocks/franka_active_control_mock.hpp>
#include <franka_hardware_mocks/franka_hardware_robot_mock.hpp>
#include <franka_hardware_mocks/franka_robot_mock.hpp>
#include "test_utils.hpp"

using namespace std::chrono_literals;

class FrankaActionServerTests
    : public ::testing::TestWithParam<
          std::pair<std::function<void(std::shared_ptr<MockRobot> mock_robot)>,
                    rclcpp_action::ResultCode>> {
 public:
  auto SetUp() -> void override {
    auto urdf_string = readFileToString(TEST_CASE_DIRECTORY + robot_type + ".urdf");
    auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    auto number_of_expected_hardware_components = 1;

    ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

    default_hardware_info = parsed_hardware_infos[0];
    default_franka_hardware_interface.on_init(default_hardware_info);
  }

  auto TearDown() -> void override {
    default_franka_hardware_interface.on_deactivate(rclcpp_lifecycle::State());
  }

 protected:
  std::string robot_type{"fr3"};
  std::shared_ptr<MockRobot> default_mock_robot = std::make_shared<MockRobot>();
  std::shared_ptr<MockFrankaRobot> default_mock_franka_robot = std::make_shared<MockFrankaRobot>();
  hardware_interface::HardwareInfo default_hardware_info;
  franka_hardware::FrankaHardwareInterface default_franka_hardware_interface{default_mock_robot,
                                                                             robot_type};
  /* Helper function to get the response of a action service */
  template <typename action_client_type>
  void get_action_service_response(
      const std::string& action_name,
      rclcpp_action::ResultCode result_code,
      const std::optional<std::function<void(std::shared_ptr<MockRobot> mock_robot)>>&
          mock_function = std::nullopt,
      const std::optional<typename action_client_type::Goal>& goal_msg_opt = std::nullopt) {
    if (mock_function.has_value()) {
      mock_function.value()(default_mock_robot);
    }

    auto node = rclcpp::Node::make_shared("test_node");

    auto client = rclcpp_action::create_client<action_client_type>(node, action_name);
    if (!client->wait_for_action_server(20s)) {
      ASSERT_TRUE(false) << "Action not available after waiting";
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    bool is_finished = false;
    auto goal_msg = typename action_client_type::Goal();
    if (goal_msg_opt.has_value()) {
      goal_msg = goal_msg_opt.value();
    }

    auto send_goal_options = typename rclcpp_action::Client<action_client_type>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&](const auto& future_result) {
      auto goal_handle = future_result.get();
      ASSERT_TRUE(goal_handle);
    };
    send_goal_options.feedback_callback = [&](auto, auto) { ASSERT_TRUE(false); };
    send_goal_options.result_callback = [&](const auto& result) {
      ASSERT_EQ(result.code, result_code);
      is_finished = true;
    };

    auto action_accepted = client->async_send_goal(goal_msg);
    auto start_point = std::chrono::system_clock::now();
    auto end_point = start_point + 5s;
    while (action_accepted.wait_for(0s) != std::future_status::ready) {
      executor.spin_some();

      ASSERT_LE(std::chrono::system_clock::now(), end_point);
    }
    auto goal_handle = action_accepted.get();

    auto result = client->async_get_result(goal_handle, send_goal_options.result_callback);
    start_point = std::chrono::system_clock::now();
    end_point = start_point + 5s;
    while (!is_finished || result.wait_for(0s) != std::future_status::ready) {
      executor.spin_some();

      ASSERT_LE(std::chrono::system_clock::now(), end_point);
    }

    ASSERT_TRUE(is_finished);
  }
};

class FrankaAutomaticRecoveryActionServerTests : public FrankaActionServerTests {};

TEST_P(FrankaAutomaticRecoveryActionServerTests,
       whenErrorRecoveryActionTriggered_thenErrorRecoveryServiceCallExecuted) {
  auto param = GetParam();

  get_action_service_response<franka_msgs::action::ErrorRecovery>("action_server/error_recovery",
                                                                  param.second, param.first);
}

INSTANTIATE_TEST_SUITE_P(
    FrankaAutomaticRecoveryActionServerTestsInstantiation,
    FrankaAutomaticRecoveryActionServerTests,
    ::testing::Values(std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery()).Times(1);
                          },
                          rclcpp_action::ResultCode::SUCCEEDED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::CommandException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::NetworkException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(
                                    franka::InvalidOperationException("not in error state")));
                          },
                          rclcpp_action::ResultCode::ABORTED)));

class FrankaPTPMotionActionServerTests : public FrankaActionServerTests {
 protected:
  franka::RobotState default_robot_state{.robot_mode = franka::RobotMode::kMove};
  std::shared_ptr<MockFrankaRobot> default_franka_robot_mock = std::make_shared<MockFrankaRobot>();
};

TEST_F(FrankaPTPMotionActionServerTests,
       whenPTPMotionThrowsFrankaException_thenActionAbortedNotCrash) {
  EXPECT_CALL(*default_mock_robot, getRobot())
      .Times(1)
      .WillOnce(testing::Throw(franka::InvalidOperationException("robot in reflex")));

  auto goal = franka_msgs::action::PTPMotion::Goal();
  goal.goal_joint_configuration = {0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.5};
  goal.maximum_joint_velocities = std::vector<double>(7, 1.0);
  goal.goal_tolerance = 0.01;

  get_action_service_response<franka_msgs::action::PTPMotion>(
      "action_server/ptp_motion", rclcpp_action::ResultCode::ABORTED, {}, goal);
}

TEST_F(FrankaPTPMotionActionServerTests,
       whenPTPMotionThrowsNetworkException_thenActionAbortedNotCrash) {
  EXPECT_CALL(*default_mock_robot, getRobot())
      .Times(1)
      .WillOnce(testing::Throw(franka::NetworkException("connection lost")));

  auto goal = franka_msgs::action::PTPMotion::Goal();
  goal.goal_joint_configuration = {0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.5};
  goal.maximum_joint_velocities = std::vector<double>(7, 1.0);
  goal.goal_tolerance = 0.01;

  get_action_service_response<franka_msgs::action::PTPMotion>(
      "action_server/ptp_motion", rclcpp_action::ResultCode::ABORTED, {}, goal);
}

TEST_F(FrankaPTPMotionActionServerTests,
       whenPTPMotionActionTriggered_thenPTPMotionServiceCallExecuted) {
  auto goal = franka_msgs::action::PTPMotion::Goal();
  goal.goal_joint_configuration = {0.0, -1.0, 0.0, -2.0, 0.0, 1.5, 0.5};
  goal.maximum_joint_velocities = std::vector<double>(7, 1.0);
  goal.goal_tolerance = 0.01;

  auto mock_active_control = std::make_unique<MockActiveControl>();
  EXPECT_CALL(*default_mock_robot, getRobot())
      .Times(1)
      .WillOnce(testing::Return(default_franka_robot_mock));
  EXPECT_CALL(*default_franka_robot_mock,
              startAsyncJointPositionControl(::testing::_, ::testing::_))
      .WillOnce(::testing::Return(::testing::ByMove(
          std::unique_ptr<franka::ActiveControlBase>(std::move(mock_active_control)))));

  std::copy(goal.goal_joint_configuration.cbegin(), goal.goal_joint_configuration.cend(),
            std::begin(default_robot_state.q_d));
  default_robot_state.dq_d = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  EXPECT_CALL(*default_mock_robot, getCurrentState())
      .WillRepeatedly(::testing::ReturnRef(default_robot_state));

  get_action_service_response<franka_msgs::action::PTPMotion>(
      "action_server/ptp_motion", rclcpp_action::ResultCode::SUCCEEDED, {}, goal);
}

TEST_F(FrankaPTPMotionActionServerTests,
       whenPTPMotionActionTriggeredWithInvalidGoal_thenAbortIsReturned) {
  EXPECT_CALL(*default_mock_robot, getRobot())
      .Times(1)
      .WillOnce(testing::Return(default_franka_robot_mock));

  auto goal = franka_msgs::action::PTPMotion::Goal();
  goal.goal_joint_configuration = {0.0, -1.0, 0.0};  // Invalid size
  goal.maximum_joint_velocities = std::vector<double>(7, 1.0);
  goal.goal_tolerance = 0.01;

  get_action_service_response<franka_msgs::action::PTPMotion>(
      "action_server/ptp_motion", rclcpp_action::ResultCode::ABORTED, {}, goal);
}
