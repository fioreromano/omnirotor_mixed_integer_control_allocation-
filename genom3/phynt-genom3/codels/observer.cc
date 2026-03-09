/*
 * Copyright (c) 2018,2023-2024 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Anthony Mallet on Mon Jan 22 2018
 */
#include "acphynt.h"

#include <cmath>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

/*
 * This file implements a wrench observer, as described in:
 *
 * T. Tomić and C. Ott and S. Haddadin, "External Wrench Estimation, Collision
 * Detection, and Reflex Reaction for Flying Robots", IEEE Transactions on
 * Robotics, vol. 33, no. 6, pp. 1467-1482, December 2017.
 */

static Eigen::Vector3d exF(Eigen::Vector3d::Zero()); /* external force */
static Eigen::Vector3d exT(Eigen::Vector3d::Zero()); /* external torque */
static Eigen::Vector3d eL(Eigen::Vector3d::Zero()); /* angular momentum */

static inline void	phynt_lpfilter(double *out, double in, double bias,
                                double thresh, double L);

void
phynt_wrench_reset()
{
  exF = exT = eL = Eigen::Vector3d::Zero();
}


int
phynt_wrench_observer(const phynt_ids_body_s *body,
                      const phynt_ids_wo_s *wo,
                      const or_pose_estimator_state *state,
                      const or_wrench_estimator_state *mwrench,
                      or_wrench_estimator_state *ewrench)
{
  using namespace Eigen;

  Quaternion<double> q; /* orientation */
  Vector3d w; /* angular velocity (world frame) */
  Vector3d a; /* acceleration (world frame) */

  Vector3d L; /* angular momentum (world frame) */

  Vector3d mf; /* measured thrust */
  Vector3d mt; /* measured torque */

  Map< const Matrix<double, 3, 3, RowMajor> > J(body->J);

  /* current state */
  if (!state->att._present || std::isnan(state->att._value.qw))
    return 1;
  else
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;

  if (!state->avel._present || std::isnan(state->avel._value.wx))
    w = Vector3d::Zero();
  else
    w << state->avel._value.wx, state->avel._value.wy, state->avel._value.wz;

  if (!state->acc._present || std::isnan(state->acc._value.ax)){
    a = Vector3d::Zero();
  }
  else {
    a << state->acc._value.ax, state->acc._value.ay, state->acc._value.az;
  }

  /* measured wrench */
  if (!mwrench->force._present || std::isnan(mwrench->force._value.x))
    return 1;
  else
    mf <<
      mwrench->force._value.x,
      mwrench->force._value.y,
      mwrench->force._value.z;

  if (!mwrench->torque._present || std::isnan(mwrench->torque._value.x))
    return 1;
  else
    mt <<
      mwrench->torque._value.x,
      mwrench->torque._value.y,
      mwrench->torque._value.z;

  /* External wrench computation.
   * Implements eq. 19 and 24 in Markus Ryll, Giuseppe Muscio, Francesco
   * Pierri, Elisabetta Cataldi, Gianluca Antonelli, et al.. 6D interaction
   * control with aerial robots: The flying end-effector paradigm. The
   * International Journal of Robotics Research, SAGE Publications, 2019, 38
   * (9), pp.1045-1062. */

  /* force estimation */
  exF +=
    Map< const Matrix<double, 3, 1> >(wo->K).asDiagonal() * (
      - exF
      + body->mass * (a + Vector3d(0, 0, 9.81))
      - mf
      ) * phynt_control_period_ms/1000.;

  /* torque estimation */
  L = q * J * (q.conjugate() * w);
  eL +=
    ( L.cross(w)
      + mt
      + exT ) * phynt_control_period_ms/1000.;

  exT =
    Map< const Matrix<double, 3, 1> >(&wo->K[3]).asDiagonal() * (L - eL);

  /* output */
  ewrench->intrinsic = false;

  ewrench->force._present = true;
  phynt_lpfilter(&ewrench->force._value.x,
                 exF[0], wo->bias[0], wo->thresh[0], wo->L[0]);
  phynt_lpfilter(&ewrench->force._value.y,
                 exF[1], wo->bias[1], wo->thresh[1], wo->L[1]);
  phynt_lpfilter(&ewrench->force._value.z,
                 exF[2], wo->bias[2], wo->thresh[2], wo->L[2]);
  ewrench->force_cov._present = false;

  ewrench->torque._present = true;
  phynt_lpfilter(&ewrench->torque._value.x,
                 exT[0], wo->bias[3], wo->thresh[3], wo->L[3]);
  phynt_lpfilter(&ewrench->torque._value.y,
                 exT[1], wo->bias[4], wo->thresh[4], wo->L[4]);
  phynt_lpfilter(&ewrench->torque._value.z,
                 exT[2], wo->bias[5], wo->thresh[5], wo->L[5]);
  ewrench->torque_cov._present = false;

  return 0;
}

static inline void
phynt_lpfilter(double *out, double in, double bias, double thresh, double L)
{
  in -= bias;
  if (std::fabs(in) < thresh) in = 0.;

  if (!std::isnan(*out))
    *out += L * (in - *out);
  else
    *out = in;
}
