#ifndef UTIL_MATH_HELPER_HPP_
#define UTIL_MATH_HELPER_HPP_

#include <Eigen/Core>

// TODO: Need tests.

/* Quaternion format : [x, y, z, w]
* q x p =
* [ q4p1 + q3p2 - q2p3 + q1p4
*  -q3p1 + q4p2 + q1p3 + q2p4
*   q2p1 - q1p2 + q4p3 + q3p4
*  -q1p1 - q2p2 - q3p3 + q4p4 ]
*
*  Most cases here is delta_q * p.
*/
inline Eigen::Vector4d quaternion_multi(const Eigen::Vector4d &q,
                                        const Eigen::Vector4d &p) {
  Eigen::Vector4d result;
  result << q(3) * p(0) + q(2) * p(1) - q(1) * p(2) + q(0) * p(3),
      -q(2) * p(0) + q(3) * p(1) + q(0) * p(2) + q(1) * p(3),
      q(1) * p(0) - q(0) * p(1) + q(3) * p(2) + q(2) * p(3),
      -q(0) * p(0) - q(1) * p(1) + q(2) * p(2) + q(3) * p(3);
  return result;
}

/*
 * Order: x, y, z, w
 */
inline Eigen::Vector4d inverse_quaternion(const Eigen::Vector4d &p) {
  Eigen::Vector4d normalized_p = p.normalized();
  Eigen::Vector4d inv_p(-normalized_p(0), -normalized_p(1), -normalized_p(2),
                        normalized_p(3));
  return inv_p;
}


inline Eigen::Vector4d conjugate_quaternion(const Eigen::Vector4d &p) {}

/*
 * Rotate vector p by rotation q
 * output = q * p * q^-1
 */
inline Eigen::Vector4d rotate_vector(const Eigen::Vector4d &p,
                                     const Eigen::Vector4d &q) {}

/*
 * [q1, q2, q3]' -->
 * [  0 -q3  q2
 *   q3   0 -q1
 *  -q2  q1   0 ]
 */
inline Eigen::MatrixXd skew_symmetric(const Eigen::Vector3d &w) {
  Eigen::MatrixXd m(3, 3);
  m << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  return m;
}

/* w = [wx, wy, wz]
 * Omega(w) -->
 * [  0  wz -wy  wx
 *  -wz   0  wx  wy
 *   wy -wx   0  wz
 *  -wx -wy -wz   0 ]
 */
inline Eigen::MatrixXd omega_matrix(const Eigen::Vector3d &w) {
  Eigen::MatrixXd m(4, 4);
  m << 0, w(2), -w(1), w(0), -w(2), 0, w(0), w(1), w(1), -w(0), 0, w(2), -w(0),
      -w(1), -w(2), 0;
  return m;
}

/*
 * q(theta, v) =
 *    cos(theta / 2), v_x * sin(theta / 2), vy * sin(theta / 2),
 *    vz * sin(theta / 2)
 */
inline Eigen::Vector4d vector_angle_to_quaternion(double angle,
                                                  const Eigen::Vector3d &v) {
  Eigen::Vector4d q;
  q << cos(angle / 2), v(0) * sin(angle / 2), v(1) * sin(angle / 2),
      v(2) * sin(angle / 2);
  return q;
}

#endif  // UTIL_MATH_HELPER_HPP_
