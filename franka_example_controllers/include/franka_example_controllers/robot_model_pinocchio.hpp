#pragma once
#include "franka_example_controllers/robot_model_interface.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <pinocchio/parsers/urdf.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

namespace robot_model_interface
{
    class RobotModelPinocchio : public RobotModelInterface
    {
        public: 
            RobotModelPinocchio(const std::string& robot_description);

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

            std::string resolveFrameName(const std::string& frame_name) const;

        
        private:
            pinocchio::Model model_;
            pinocchio::Data data_;
            Vector7d q_;
            Vector7d dq_;
    };
}


