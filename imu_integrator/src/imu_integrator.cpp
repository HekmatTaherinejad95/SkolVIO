#include "imu_integrator.hpp"

#include <cmath>

#include "math_helper.hpp"

namespace vio {

/*
 * Follows paper : "Indirect Kalman filter for 3D Attitude Estimation" - Eq 123
 * |start_q| is the current orientation in quaternion [ x, y, z, w],
 * |w| is the angular velocity in [ wx, wy, wz ]
 * |delta_t| is the time between current orientation and timestamp of |w|,
 * |end_q| is the predicted orientation at timestamp of |w|
 */
bool ImuIntegrator::ZerothOrderIntegration(const Eigen::Vector4d &start_q,
                                           const Eigen::Vector3d &w,
                                           double delta_t,
                                           Eigen::Vector4d &end_q) {
  // TODO: Is sin/cos in <cmath> ok to use?
  // TODO: Numerical stability.
  const double tmp = w.norm() / 2.0 * delta_t;
  Eigen::Vector4d delta_q;
  delta_q << w *w.norm() * std::sin(tmp), w.norm() * std::cos(tmp);

  end_q = quaternion_multi(delta_q, start_q);

  return true;
}

}  // vio
