#pragma once

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/loaned_state_interface.hpp>
#include <Eigen/Eigen>

namespace robot_model_interface
{
    using Vector7d = Eigen::Matrix<double, 7, 1>;

    class RobotModelInterface{
        public: 
            virtual ~RobotModelInterface() = default;
            virtual void updateState(const Vector7d& q,const Vector7d& dq) = 0;

            virtual Eigen::Affine3d getPose(const std::string& frame_name) = 0;
            virtual Eigen::Matrix<double, 6, 7> getJacobian(const std::string& frame_name) = 0;
            virtual Eigen::Matrix<double, 7, 7> getMassMatrix() = 0;
            virtual Vector7d getCoriolis() = 0;
            virtual Vector7d getGravity() = 0;

            virtual std::vector<std::string> get_state_interface_names() = 0;
            virtual void assign_loaned_state_interfaces(
                std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) = 0;
            virtual void release_interfaces() = 0;

    };
}