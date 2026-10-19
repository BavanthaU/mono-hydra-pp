#include "ptc_depth/triangulation.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

int main() {
  ptc_depth::CameraIntrinsics cam(120, 160, 100, 100, 80, 60);
  ptc_depth::Triangulator tri(cam);
  Eigen::Matrix3d R = Eigen::AngleAxisd(0.12, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Vector3d t(-0.2, 0.01, 0.02);
  ptc_depth::Correspondences m;
  double expected = 0;
  Eigen::Vector3d p(0.1, 0.2, 3.0), q = R * p + t;
  m.u0.push_back(100 * p.x() / p.z() + 80);
  m.v0.push_back(100 * p.y() / p.z() + 60);
  m.u1.push_back(100 * q.x() / q.z() + 80);
  m.v1.push_back(100 * q.y() / q.z() + 60);
  expected = q.z();
  auto result = tri.triangulate(m, R.transpose(), -R.transpose() * t);
  const int u = std::lround(m.u1[0]), v = std::lround(m.v1[0]);
  double z = result.z1_tri.at<float>(v, u), rho = result.rho.at<float>(v, u);
  std::cout << "depth=" << z << " expected=" << expected << " Sampson=" << rho << std::endl;
  return std::isfinite(z) && std::abs(z - expected) < 1e-4 && rho < 1e-8 ? 0 : 1;
}
