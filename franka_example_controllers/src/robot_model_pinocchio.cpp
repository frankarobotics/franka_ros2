#include "franka_example_controllers/robot_model_pinocchio.hpp"
#include <unordered_map>


namespace robot_model_interface
{
    RobotModelPinocchio::RobotModelPinocchio(const std::string& robot_description)
    {
        pinocchio::urdf::buildModelFromXML(robot_description, model_);
        model_.gravity.linear() = Eigen::Vector3d(0, 0, -9.81);
        data_ = pinocchio::Data(model_);
    }

    void RobotModelPinocchio::updateState(const Vector7d& q,const Vector7d& dq)
    {
        q_ = q;
        dq_ = dq;
        pinocchio::computeAllTerms(model_, data_, q_, dq_);
        pinocchio::updateFramePlacements(model_, data_);
    }

    Eigen::Matrix<double, 7, 7> RobotModelPinocchio::getMassMatrix()
    {
        return Eigen::Matrix<double,7,7>(data_.M);
    }

    Vector7d RobotModelPinocchio::getCoriolis()
    {
        return data_.nle - data_.g;
    }

    Vector7d RobotModelPinocchio::getGravity()
    {
        return data_.g;
    }

    std::vector<std::string> RobotModelPinocchio::get_state_interface_names()
    {
        return {};
    }

    void RobotModelPinocchio::assign_loaned_state_interfaces(
        std::vector<hardware_interface::LoanedStateInterface>& state_interfaces)
    {
        // No state interfaces to assign for Pinocchio model
    }

    void RobotModelPinocchio::release_interfaces()
    {
        // No interfaces to release for Pinocchio model
    }

    std::string RobotModelPinocchio::resolveFrameName(const std::string& frame_name) const
    {
        static const std::unordered_map<std::string, std::string> frame_map = {
            {"joint1",       "fr3_link1"},
            {"joint2",       "fr3_link2"},
            {"joint3",       "fr3_link3"},
            {"joint4",       "fr3_link4"},
            {"joint5",       "fr3_link5"},
            {"joint6",       "fr3_link6"},
            {"joint7",       "fr3_link7"},
            {"end_effector", "fr3_link8"},
        };
        auto it = frame_map.find(frame_name);
        return (it != frame_map.end()) ? it->second : frame_name;
    }

    Eigen::Affine3d RobotModelPinocchio::getPose(const std::string& frame_name)
    {
        auto frame_id = model_.getFrameId(resolveFrameName(frame_name));
        return Eigen::Affine3d(data_.oMf[frame_id].toHomogeneousMatrix());
    }

    Eigen::Matrix<double, 6, 7> RobotModelPinocchio::getJacobian(const std::string& frame_name)
    {
        auto frame_id = model_.getFrameId(resolveFrameName(frame_name));
        Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();
        pinocchio::getFrameJacobian(model_, data_, frame_id,
            pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }
}
