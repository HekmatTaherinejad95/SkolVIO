#ifndef IMU_INTEGRATOR_IMU_INTEGRATOR_HPP_
#define IMU_INTEGRATOR_IMU_INTEGRATOR_HPP_

// TODO: Figure out which header is the minimum to include.
#include <Eigen/Core>

namespace vio {

class ImuIntegrator {
 public:
  ImuIntegrator() {}

  bool ZerothOrderIntegration(const Eigen::Vector4d &start_q,
                              const Eigen::Vector3d &w, double delta_t,
                              Eigen::Vector4d &end_q);

 private:
};

}  // vio

#endif
