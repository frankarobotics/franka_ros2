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

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_move_to_start_example_controller.hpp"

#include <ros2_control_test_assets/descriptions.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/utilities.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

const double k_EPS = 1e-5;

void MoveToStartExampleControllerTest::SetUpTestSuite() {
  rclcpp::init(0, nullptr);
}

void MoveToStartExampleControllerTest::TearDownTestSuite() {
  rclcpp::shutdown();
}

void MoveToStartExampleControllerTest::SetUp() {
  controller_ = std::make_unique<franka_example_controllers::MoveToStartExampleController>();
}

void MoveToStartExampleControllerTest::TearDown() {
  controller_.reset(nullptr);
}

void MoveToStartExampleControllerTest::SetUpController() {
  const auto result = controller_->init("test_move_to_start_example",
                                        ros2_control_test_assets::minimal_robot_urdf, 30, "",
                                        rclcpp::NodeOptions()
                                            .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true));
  ASSERT_EQ(result, controller_interface::return_type::OK);
  std::vector<LoanedCommandInterface> command_ifs;
  std::vector<LoanedStateInterface> state_ifs;

  command_ifs.emplace_back(joint_1_pos_cmd_);
  command_ifs.emplace_back(joint_2_pos_cmd_);
  command_ifs.emplace_back(joint_3_pos_cmd_);
  command_ifs.emplace_back(joint_4_pos_cmd_);
  command_ifs.emplace_back(joint_5_pos_cmd_);
  command_ifs.emplace_back(joint_6_pos_cmd_);
  command_ifs.emplace_back(joint_7_pos_cmd_);

  state_ifs.emplace_back(joint_1_pos_state_);
  state_ifs.emplace_back(joint_1_vel_state_);
  state_ifs.emplace_back(joint_2_pos_state_);
  state_ifs.emplace_back(joint_2_vel_state_);
  state_ifs.emplace_back(joint_3_pos_state_);
  state_ifs.emplace_back(joint_3_vel_state_);
  state_ifs.emplace_back(joint_4_pos_state_);
  state_ifs.emplace_back(joint_4_vel_state_);
  state_ifs.emplace_back(joint_5_pos_state_);
  state_ifs.emplace_back(joint_5_vel_state_);
  state_ifs.emplace_back(joint_6_pos_state_);
  state_ifs.emplace_back(joint_6_vel_state_);
  state_ifs.emplace_back(joint_7_pos_state_);
  state_ifs.emplace_back(joint_7_vel_state_);

  controller_->assign_interfaces(std::move(command_ifs), std::move(state_ifs));
}

TEST_F(MoveToStartExampleControllerTest, controller_gains_not_set_failure) {
  SetUpController();

  // Failure due to not set K, D gains
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, contoller_gain_empty) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", std::vector<double>()});

  // Failure due empty k gain parameter
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, contoller_damping_gain_empty) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", std::vector<double>()});

  // Failure due to empty d gain parameter
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::FAILURE);
}

TEST_F(MoveToStartExampleControllerTest, correct_controller_gains_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MoveToStartExampleControllerTest, correct_setup_on_activate_expect_success) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
}

TEST_F(MoveToStartExampleControllerTest, correct_setup_on_update_expect_ok) {
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);

  ASSERT_EQ(controller_->update(time, duration), controller_interface::return_type::OK);

  EXPECT_NEAR(joint_1_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
  EXPECT_NEAR(joint_2_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
  EXPECT_NEAR(joint_3_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
  EXPECT_NEAR(joint_4_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
  EXPECT_NEAR(joint_5_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
  EXPECT_NEAR(joint_6_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
  EXPECT_NEAR(joint_7_pos_cmd_.get_optional<double>().value_or(-1), 0.0, k_EPS);
}

TEST_F(MoveToStartExampleControllerTest, process_finished_published_off_realtime_thread_once) {
  // Exact goal: the motion is finished on the first update (the default
  // fixture misses joint 4 by ~4.5e-6 > kDeltaQMotionFinished).
  joint_q_state_ = {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // The fixture joint state already equals the goal, so the motion is finished
  // on the first update. Extra realtime cycles must not touch the parameter.
  auto time = rclcpp::Time(0);
  auto duration = rclcpp::Duration(0, 0);
  constexpr int kExtraCycles = 10;
  for (int i = 0; i < kExtraCycles; ++i) {
    ASSERT_EQ(controller_->update(time, duration), controller_interface::return_type::OK);
  }
  EXPECT_FALSE(controller_->get_node()->get_parameter("process_finished").as_bool());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!controller_->get_node()->get_parameter("process_finished").as_bool() &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(controller_->get_node()->get_parameter("process_finished").as_bool());

  // Exactly once: externally clearing the parameter must not re-publish it,
  // no matter how many more realtime cycles run or timer periods elapse.
  controller_->get_node()->set_parameter({"process_finished", false});
  for (int i = 0; i < kExtraCycles; ++i) {
    ASSERT_EQ(controller_->update(time, duration), controller_interface::return_type::OK);
  }
  const auto settle_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (std::chrono::steady_clock::now() < settle_deadline) {
    rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(controller_->get_node()->get_parameter("process_finished").as_bool());
}

namespace {
// Polls the parameter until it reads the expected value (a background spinner pumps
// the wall timer); returns whether it did. No timing assertion — the generous
// deadline is a hang guard only.
bool waitForProcessFinished(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                            bool expected,
                            std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  bool last_value = !expected;
  while (std::chrono::steady_clock::now() < deadline) {
    last_value = node->get_parameter("process_finished").as_bool();
    if (last_value == expected) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return last_value == expected;
}

// Pumps the controller's wall timer on its own thread, so timer-executed writes are
// attributable to a thread other than the test (realtime-simulating) thread.
class BackgroundSpinner {
 public:
  explicit BackgroundSpinner(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base) {
    spinner_.add_node(node_base);
    spinner_thread_ = std::thread([this]() { spinner_.spin(); });
  }
  ~BackgroundSpinner() {
    spinner_.cancel();
    spinner_thread_.join();
  }

 private:
  rclcpp::executors::SingleThreadedExecutor spinner_;
  std::thread spinner_thread_;
};
}  // namespace

TEST_F(MoveToStartExampleControllerTest, reactivation_resets_and_republishes_process_finished) {
  // Exact goal: the motion finishes on the first update.
  joint_q_state_ = {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  const BackgroundSpinner spinner(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  EXPECT_TRUE(waitForProcessFinished(controller_->get_node(), true, std::chrono::seconds(5)));

  // Re-activation must reset to false (no stale true latched across the reset),
  // then report true again once the motion re-finishes.
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_TRUE(waitForProcessFinished(controller_->get_node(), false, std::chrono::seconds(2)));

  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  EXPECT_TRUE(waitForProcessFinished(controller_->get_node(), true, std::chrono::seconds(5)));
}

TEST_F(MoveToStartExampleControllerTest,
       reconfigure_after_finish_reports_false_until_motion_finishes) {
  // Exact goal: the motion finishes on the first update.
  joint_q_state_ = {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  const BackgroundSpinner spinner(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  EXPECT_TRUE(waitForProcessFinished(controller_->get_node(), true, std::chrono::seconds(5)));

  // Full lifecycle back through cleanup and configure again: the previous run's true
  // must not resurface. Nothing runs update() below, so only a stale requested flag
  // meeting the fresh timer could flip it — several 20 ms periods leave that nowhere
  // to hide. (The 150 ms is a settle window, not a timing claim: the value must hold.)
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_cleanup(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_FALSE(controller_->get_node()->get_parameter("process_finished").as_bool());

  // The reconfigured controller still runs: activate and finish reports true again.
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  EXPECT_TRUE(waitForProcessFinished(controller_->get_node(), true, std::chrono::seconds(5)));
}

TEST_F(MoveToStartExampleControllerTest, no_parameter_write_from_realtime_thread) {
  // Exact goal: the motion finishes on the first update.
  joint_q_state_ = {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};
  SetUpController();
  controller_->get_node()->set_parameter({"k_gains", K_gains_});
  controller_->get_node()->set_parameter({"d_gains", D_gains_});

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // Arm the recorder after all setup-thread writes: every set_parameter from here
  // on records its calling thread. on_activate()/update() run on this thread below
  // (simulating the realtime thread); only the timer may write.
  std::mutex writers_mutex;
  std::set<std::thread::id> writer_threads;
  auto recorder = controller_->get_node()->add_on_set_parameters_callback(
      [&](const std::vector<rclcpp::Parameter>&) {
        const std::lock_guard<std::mutex> guard(writers_mutex);
        writer_threads.insert(std::this_thread::get_id());
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  (void)recorder;

  // The timer must run on its own thread: spin_some would execute it inline here and
  // hide the thread the write comes from.
  const BackgroundSpinner spinner(controller_->get_node()->get_node_base_interface());

  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration(0, 0)),
            controller_interface::return_type::OK);
  const bool published =
      waitForProcessFinished(controller_->get_node(), true, std::chrono::seconds(5));

  EXPECT_TRUE(published);
  const std::lock_guard<std::mutex> guard(writers_mutex);
  EXPECT_EQ(writer_threads.size(), 1U);
  EXPECT_EQ(writer_threads.count(std::this_thread::get_id()), 0U);
}
