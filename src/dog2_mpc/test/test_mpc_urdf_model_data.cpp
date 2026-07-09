#include "dog2_dynamics/dog2_model.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string shellQuote(const std::string& value) {
  std::string quoted = "'";
  for (char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

std::string runXacro(const std::string& xacro_path, const std::string& controllers_yaml) {
  const std::string command =
    "xacro " + shellQuote(xacro_path) +
    " controllers_yaml:=" + shellQuote(controllers_yaml);

  FILE* pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    throw std::runtime_error("Failed to run xacro command");
  }

  std::array<char, 4096> buffer{};
  std::string output;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }

  const int status = pclose(pipe);
  if (status != 0 || output.empty()) {
    std::ostringstream msg;
    msg << "xacro failed with status " << status << ": " << command;
    throw std::runtime_error(msg.str());
  }
  return output;
}

void requireNear(double actual, double expected, double tol, const std::string& label) {
  if (std::abs(actual - expected) > tol) {
    std::ostringstream msg;
    msg << label << " expected " << expected << " got " << actual;
    throw std::runtime_error(msg.str());
  }
}

}  // namespace

int main() {
  const std::string share_dir =
    ament_index_cpp::get_package_share_directory("dog2_description");
  const std::string urdf_xml = runXacro(
    share_dir + "/urdf/dog2_symmetric.urdf.xacro",
    share_dir + "/config/ros2_controllers.yaml");

  auto model = dog2_dynamics::Dog2Model::fromUrdfXml(urdf_xml);
  requireNear(model.mass(), 12.0028, 1e-4, "symmetric total mass");

  const auto& pin_model = model.getModel();
  const auto base_frame_id = pin_model.getFrameId("base_link");
  const Eigen::Matrix3d base_inertia =
    pin_model.frames[base_frame_id].inertia.inertia().matrix();
  requireNear(base_inertia(0, 0), 0.0153, 1e-8, "base_link ixx");
  requireNear(base_inertia(1, 1), 0.044, 1e-8, "base_link iyy");
  requireNear(base_inertia(2, 2), 0.052, 1e-8, "base_link izz");

  const Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(model.nq());
  const Eigen::Vector3d lf = model.footPosition("lf_foot_link", q_zero);
  const Eigen::Vector3d lh = model.footPosition("lh_foot_link", q_zero);
  const Eigen::Vector3d rh = model.footPosition("rh_foot_link", q_zero);
  const Eigen::Vector3d rf = model.footPosition("rf_foot_link", q_zero);

  requireNear(lf.x(), -0.103323, 1e-6, "lf foot x");
  requireNear(lh.x(), 0.172927, 1e-6, "lh foot x");
  requireNear(rh.x(), 0.172927, 1e-6, "rh foot x");
  requireNear(rf.x(), -0.103323, 1e-6, "rf foot x");
  requireNear(lf.y(), -0.1184, 1e-6, "lf foot y");
  requireNear(rf.y(), 0.1184, 1e-6, "rf foot y");

  if (std::string(dog2_dynamics::Dog2Model::FOOT_NAMES[0]) == "lf_foot_link") {
    throw std::runtime_error("Dog2Model::FOOT_NAMES unexpectedly matches MPC order; keep explicit mapping test updated");
  }

  std::cout << "[PASS] MPC URDF-derived symmetric mass/inertia/foot-order data validated"
            << std::endl;
  return 0;
}
