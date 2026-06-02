#pragma once 

#include "franka_example_controllers/robot_model_interface.hpp"
#include <franka_semantic_components/franka_robot_model.hpp>

namespace robot_model_interface
{
    class RobotModelFranka : public RobotModelInterface
    {
        public: 
            RobotModelFranka(const std::string& robot_type);

            void updateState(const Vector7d& q,const Vector7d& dq) override;

            Eigen::Affine3d getPose(const std::string& frame_name) override;
            Eigen::Matrix<double, 6, 7> getJacobian(const std::string& frame_name) override;
            Eigen::Matrix<double, 7, 7> getMassMatrix() override;
            Vector7d getCoriolis() override;
            Vector7d getGravity() override;

            std::vector<std::string> get_state_interface_names() override;
            void assign_loaned_state_interfaces(
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) override;
            void release_interfaces() override;

        
        private:
            std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
            Vector7d q_;
            Vector7d dq_;
            const std::string k_robot_state_interface_name{"robot_state"};
            const std::string k_robot_model_interface_name{"robot_model"};

                franka::Frame stringToFrankaFrame(const std::string& frame_name) const;
    };
}