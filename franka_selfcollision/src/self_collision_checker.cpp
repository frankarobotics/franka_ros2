// Copyright (c) 2026 Franka Robotics GmbH
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

#include "franka_selfcollision/self_collision_checker.hpp"

// Pinocchio headers
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace franka_selfcollision {

SelfCollisionChecker::SelfCollisionChecker(const std::string& urdf_xml,
                                           const std::string& srdf_xml,
                                           double security_margin,
                                           rclcpp::Logger logger,
                                           rclcpp::Clock::SharedPtr clock)
    : logger_(logger), clock_(clock) {
  pinocchio::urdf::buildModelFromXML(urdf_xml, model_);

  std::istringstream urdf_stream(urdf_xml);
  pinocchio::urdf::buildGeom(model_, urdf_stream, pinocchio::COLLISION, geom_model_);

  geom_model_.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairsFromXML(model_, geom_model_, srdf_xml);

  data_ = std::make_shared<pinocchio::Data>(model_);
  geom_data_ = std::make_shared<pinocchio::GeometryData>(geom_model_);

  // Apply security margin
  for (auto& collision_request : geom_data_->collisionRequests) {
    collision_request.security_margin = security_margin;
    collision_request.enable_contact = true;
  }
}

bool SelfCollisionChecker::checkCollision(const std::vector<double>& joint_configuration,
                                          bool print_collisions) {
  if (joint_configuration.size() != (size_t)model_.nq) {
    RCLCPP_FATAL(logger_, "Dimension mismatch...");
    throw std::invalid_argument("Joint configuration dimension mismatch");
  }
  Eigen::Map<const Eigen::VectorXd> q(joint_configuration.data(), joint_configuration.size());

  return checkCollisions(q, print_collisions);
}

bool SelfCollisionChecker::checkCollisions(const Eigen::VectorXd& q, bool print_collisions) {
  bool stop_at_first = !print_collisions;

  bool collision_found =
      pinocchio::computeCollisions(model_, *data_, geom_model_, *geom_data_, q, stop_at_first);

  if (collision_found && print_collisions) {
    for (size_t k = 0; k < geom_model_.collisionPairs.size(); ++k) {
      const pinocchio::CollisionPair& cp = geom_model_.collisionPairs[k];
      const hpp::fcl::CollisionResult& cr = geom_data_->collisionResults[k];

      if (cr.isCollision()) {
        const std::string& name1 = geom_model_.geometryObjects[cp.first].name;
        const std::string& name2 = geom_model_.geometryObjects[cp.second].name;

        RCLCPP_WARN_THROTTLE(logger_, *clock_, 500, "COLLISION:  %s <--> %s", name1.c_str(),
                             name2.c_str());
      }
    }
  }

  return collision_found;
}

}  // namespace franka_selfcollision