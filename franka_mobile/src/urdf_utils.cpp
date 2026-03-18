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

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include "urdf_utils.hpp"

namespace franka_mobile {

SE3 getSe3FromDescription(const std::string& robot_description,
                          const std::string& reference_frame,
                          const std::string& target_frame) {
  KDL::Tree tree;
  kdl_parser::treeFromString(robot_description, tree);

  KDL::Chain chain;
  tree.getChain(reference_frame, target_frame, chain);

  KDL::Frame transform = KDL::Frame::Identity();
  for (const KDL::Segment& segment : chain.segments) {
    transform = transform * segment.getFrameToTip();
  }

  SE3 result;
  result.p.x() = transform.p.x();
  result.p.y() = transform.p.y();
  result.p.z() = transform.p.z();
  transform.M.GetQuaternion(result.q.x(), result.q.y(), result.q.z(), result.q.w());

  return result;
}

double getWheelRadiusFromDescription(const std::string& robot_description,
                                     const std::string& link_name) {
  urdf::Model urdf_model;
  urdf_model.initString(robot_description);

  auto wheel_link = urdf_model.getLink(link_name);
  if (wheel_link == nullptr) {
    throw std::invalid_argument("Link name" + link_name + " does not exist in robot description");
  }
  auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(
      wheel_link->collision->geometry);  // or ->visual->geometry, should be the same!!
  return cylinder->radius;
}

}  // namespace franka_mobile