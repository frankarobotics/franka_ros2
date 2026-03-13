#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include "utils.hpp"

namespace franka_example_controllers {

SE3 get_se3_from_description(const std::string& robot_description,
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

double get_wheel_radius_from_description(const std::string& robot_description,
                                         const std::string& link_name) {
  urdf::Model urdf_model;
  urdf_model.initString(robot_description);

  auto wheel_link = urdf_model.getLink(link_name);
  auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(
      wheel_link->collision->geometry);  // or ->visual->geometry, should be the same!!
  return cylinder->radius;
}

}  // namespace franka_example_controllers