/*
 * Copyright (c) 2018-2019,2023-2024 LAAS/CNRS
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
 *                                           Anthony Mallet on Wed Jan 24 2018
 */
#include "acphynt.h"

#include <err.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/Dense"

#include "codels.h"

static Eigen::Matrix3d iJ;	/* inverse apparent inertia */

static Eigen::Vector3d xf{0., 0., 0.}, vf{0., 0., 0.};
static Eigen::Vector3d wf{0., 0., 0.};
static Eigen::Quaternion<double> qf(Eigen::Quaternion<double>::Identity());

/*
 * --- phynt_admittance_reset ----------------------------------------------
 *
 * Reset filter state
 */

void
phynt_admittance_reset()
{
  using namespace Eigen;

  xf = vf = wf = Vector3d::Zero();
  qf = Quaternion<double>::Identity();
}


/*
 * --- phynt_adm_filter -----------------------------------------------------
 *
 * Implements admittance filter on the desired state.
 */

int
phynt_admittance_filter(const phynt_ids_body_s *body, const phynt_ids_af_s *afp,
                        const or_rigid_body_state *reference,
                        const or_pose_estimator_state *state,
                        const or_wrench_estimator_state *ewrench,
                        or_rigid_body_state *desired,
                        phynt_log_s *log)
{
  using namespace Eigen;

  static const double dt = phynt_control_period_ms / 1000.;

  /* filter state */
  Vector3d af, dwf;
  Matrix3d Rf;

  /* reference */
  Vector3d xr, vr, wr, ar;
  Quaternion<double> qr;
  Matrix3d Rr;

  /* external wrench */
  Vector3d exF, exT, exT_b;

  /* controller */
  Vector3d ex, eR, ev, ew;
  Matrix3d E;

  Map< const Array<double, 6, 1> > B(afp->B);
  Map< const Array<double, 6, 1> > K(afp->K);
  const Vector3d force(afp->force.x, afp->force.y, afp->force.z);
  const Vector3d torque(afp->torque.x, afp->torque.y, afp->torque.z);

  /* end effector */
  Map< const Matrix<double, 3, 3, RowMajor> > ee_or(body->ee_or);
  const Vector3d ee_pos(body->ee_pos[0], body->ee_pos[1], body->ee_pos[2]);

  Quaternion<double> q; /* orientation */
  Matrix3d R;

  /* current orientation */
  q.coeffs() << state->att._value.qx, state->att._value.qy, 
                state->att._value.qz, state->att._value.qw;

  R = q.matrix();

  /* reference attitude */
  qr.coeffs() <<
    reference->att._value.qx,
    reference->att._value.qy,
    reference->att._value.qz,
    reference->att._value.qw;

  /* external wrench */
  if (ewrench->force._present) {
    exF <<
      ewrench->force._value.x,
      ewrench->force._value.y,
      ewrench->force._value.z;
  } else {
    exF = Vector3d::Zero();
  }

  if (ewrench->torque._present) {
    exT_b <<
      ewrench->torque._value.x,
      ewrench->torque._value.y,
      ewrench->torque._value.z;
    exT = exT_b - (R * ee_pos).cross(exF);
  } else {
    exT = Vector3d::Zero();
  }

  /* position offset */
  ex = xf - Vector3d::Zero();

  /* orientation offset */
  Rf = qf.matrix();
  E = 0.5 * (Rf - Rf.transpose());
  eR <<
    (E(2, 1) - E(1, 2))/2.,
    (E(0, 2) - E(2, 0))/2.,
    (E(1, 0) - E(0, 1))/2.;

  /* velocity offset */
  ev = vf - Vector3d::Zero();
  ew = wf - Vector3d::Zero();


  /* admittance acceleration */
  af =
    1/afp->mass * (
      (- K.block<3, 1>(0, 0) * ex.array()).matrix() +
      (- B.block<3, 1>(0, 0) * ev.array()).matrix() +
      exF +
      force);

  dwf =
    iJ * Vector3d(
      (- K.block<3, 1>(3, 0) * eR.array()).matrix() +
      (- B.block<3, 1>(3, 0) * ew.array()).matrix() +
      exT +
      torque
      );

  /* integration */
  xf += vf * dt + af * dt*dt/2;
  vf += af * dt;
  wf += dwf * dt;

  Quaternion<double> dq;
  double a2 = dt * dt * wf.squaredNorm();
  if (a2 < 1e-1) {
    dq.w() = 1 - a2/8 /*std::cos(a/2)*/;
    dq.vec() = (0.5 - a2/48 /*std::sin(a/2)/a*/) * dt * wf;
  } else {
    double a = std::sqrt(a2);
    dq.w() = std::cos(a/2);
    dq.vec() = std::sin(a/2)/a * dt * wf;
  }
  qf = dq * qf;

  /* output is reference + filter (attitude quaternion needs composition) */
  *desired = *reference;

  Quaternion<double> qd(qf * qr); /* compose offset with reference */
  Matrix3d Rd_e, Rd_b;
  Rd_e = qd.matrix();
  Rd_b = Rd_e * ee_or.transpose();
  Quaternion<double> qb(Rd_b);

  desired->pos._present = true;
  desired->pos._value.x += xf(0);
  desired->pos._value.y += xf(1);
  desired->pos._value.z += xf(2); 
  
  desired->att._present = true;
  desired->att._value.qx = qd.x();
  desired->att._value.qy = qd.y();
  desired->att._value.qz = qd.z();
  desired->att._value.qw = qd.w();
  
  desired->avel._present = true;
  Vector3d w(desired->intrinsic ? qb.conjugate() * wf : wf);
  desired->avel._value.wx += w(0);
  desired->avel._value.wy += w(1);
  desired->avel._value.wz += w(2);

  desired->vel._present = true;
  Vector3d v(desired->intrinsic ? qb.conjugate() * vf : vf); 
  desired->vel._value.vx += v(0);
  desired->vel._value.vy += v(1);
  desired->vel._value.vz += v(2);  

  desired->aacc._present = true;
  Vector3d dw(desired->intrinsic ? qb.conjugate() * dwf : dwf);
  desired->aacc._value.awx += dw(0);
  desired->aacc._value.awy += dw(1);
  desired->aacc._value.awz += dw(2);

  desired->acc._present = true;
  Vector3d a(desired->intrinsic ? qb.conjugate() * af : af);
  desired->acc._value.ax += a(0);
  desired->acc._value.ay += a(1);
  desired->acc._value.az += a(2);

  /* logging */
  if (log->req.aio_fildes >= 0) {
    log->total++;
    if (log->total % log->decimation == 0) {
      if (log->pending) {
        if (aio_error(&log->req) != EINPROGRESS) {
          log->pending = false;
          if (aio_return(&log->req) <= 0) {
            warn("log");
            close(log->req.aio_fildes);
            log->req.aio_fildes = -1;
          }
        } else {
          log->skipped = true;
          log->missed++;
        }
      }
    }

    if (log->req.aio_fildes >= 0 && !log->pending) {
      double roll, pitch, yaw;
      double a, b;

      a = atan2(qd.x() + qd.z(), qd.w() - qd.y());
      b = atan2(qd.x() - qd.z(), qd.w() + qd.y());

      roll = a + b;
      pitch = asin(2 * (qd.w()*qd.y() - qd.z()*qd.x()));
      yaw = a - b;

      if (roll > M_PI) roll -= 2*M_PI;
      else if (roll < -M_PI) roll += 2*M_PI;

      if (pitch > M_PI) pitch -= 2*M_PI;
      else if (pitch < -M_PI) pitch += 2*M_PI;

      log->req.aio_nbytes = snprintf(
        log->buffer, sizeof(log->buffer),
        "%s" phynt_log_fmt "\n",
        log->skipped ? "\n" : "",
        ewrench->ts.sec, ewrench->ts.nsec,
        ewrench->force._present ? ewrench->force._value.x : nan(""),
        ewrench->force._present ? ewrench->force._value.y : nan(""),
        ewrench->force._present ? ewrench->force._value.z : nan(""),

        ewrench->torque._present ? ewrench->torque._value.x : nan(""),
        ewrench->torque._present ? ewrench->torque._value.y : nan(""),
        ewrench->torque._present ? ewrench->torque._value.z : nan(""),

        ewrench->torque._present ? exT(0) : nan(""),
        ewrench->torque._present ? exT(1) : nan(""),
        ewrench->torque._present ? exT(2) : nan(""),        

        desired->pos._value.x,
        desired->pos._value.y,
        desired->pos._value.z,
        roll, pitch, yaw,

        desired->vel._value.vx,
        desired->vel._value.vy,
        desired->vel._value.vz,

        desired->avel._value.wx,
        desired->avel._value.wy,
        desired->avel._value.wz,

        desired->acc._value.ax,
        desired->acc._value.ay,
        desired->acc._value.az,

        desired->aacc._value.awx,
        desired->aacc._value.awy,
        desired->aacc._value.awz);

      if (aio_write(&log->req)) {
        warn("log");
        close(log->req.aio_fildes);
        log->req.aio_fildes = -1;
      } else
        log->pending = true;

      log->skipped = false;
    }
  }

  phynt_kinematics_ee_to_body(body, state, desired);
  
  return 0;
}


/*
 * --- phynt_reference_check -----------------------------------------------
 *
 * Update missing fields of the reference by integrating other fields
 */

void
phynt_reference_check(const struct timeval now,
                      or_rigid_body_state *reference)
{
  static const double dt = phynt_control_period_ms / 1000.;
  static const double dt2 = dt * dt;
  static const double dt2_2 = dt2 / 2.;

  or_t3d_pos *p = &reference->pos._value;
  or_t3d_att *q = &reference->att._value;
  or_t3d_vel *v = &reference->vel._value;
  or_t3d_avel *w = &reference->avel._value;
  or_t3d_acc *a = &reference->acc._value;
  or_t3d_aacc *aw = &reference->aacc._value;

  /* deal with obsolete reference */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      0.5 + reference->ts.sec + 1e-9 * reference->ts.nsec) {
    reference->vel._present = true;
    v->vx = v->vy = v->vz = 0.;

    reference->avel._present = true;
    w->wx = w->wy = w->wz = 0.;

    reference->acc._present = true;
    a->ax = a->ay = a->az = 0.;
  }

  if (!reference->pos._present) {
    /* use previous pos and integrate vel, acc */
    Eigen::Vector3d dp(dt * v->vx + dt2_2 * a->ax,
                       dt * v->vy + dt2_2 * a->ay,
                       dt * v->vz + dt2_2 * a->az);
    if (reference->intrinsic)
      dp = Eigen::Quaternion<double>(q->qw, q->qx, q->qy, q->qz) * dp;

    p->x += dp(0);
    p->y += dp(1);
    p->z += dp(2);
  }

  if (!reference->vel._present) {
    /* use previous vel and integrate acc */
    v->vx += dt * a->ax;
    v->vy += dt * a->ay;
    v->vz += dt * a->az;
  }

  if (!reference->acc._present) {
    /* reset */
    a->ax = a->ay = a->az = 0.;
  }

  if (!reference->att._present) {
    /* use previous att and integrate avel */
    double a2 = dt2 * (w->wx * w->wx + w->wy * w->wy + w->wz * w->wz);

    if (a2 < 1e-1 /* otherwise do nothing, too high */) {
      Eigen::Quaternion<double> Q(q->qw, q->qx, q->qy, q->qz), dq;
      Eigen::Vector3d W(w->wx, w->wy, w->wz);
      if (reference->intrinsic)
        W = Q * W;

      dq.w() = 1 - a2/8; /* cos(a/2) */
      dq.vec() = dt * (0.5 - a2/48) /* dt * sin(a/2)/a */ * W;
      Q = dq * Q;

      q->qw = Q.w(); q->qx = Q.x(); q->qy = Q.y(); q->qz = Q.z();
    }
  }

  if (!reference->avel._present) {
    /* reset */
    w->wx = w->wy = w->wz = 0.;
  }

  if (!reference->aacc._present) {
    /* reset */
    aw->awx = aw->awy = aw->awz = 0.;
  }
}


/*
 * --- phynt_adm_J ----------------------------------------------------------
 *
 */

void
phynt_admittance_J(const double J[3 * 3])
{
  using namespace Eigen;

  Map< const Matrix<double, 3, 3, RowMajor> > J_(J);

  iJ = J_.inverse();
}


/*
 * --- phynt_kinematics_ee_to_body -----------------------------------------------------
 *
 * Transforms end-effector trajectories to body (CoM) trajectories
 */

void
phynt_kinematics_ee_to_body(const phynt_ids_body_s *body,
                            const or_pose_estimator_state *state,
                            or_rigid_body_state *desired)
{
  using namespace Eigen;

  /* end effector params */
  Map< const Matrix<double, 3, 3, RowMajor> > ee_or(body->ee_or);
  const Vector3d ee_pos(body->ee_pos[0], body->ee_pos[1], body->ee_pos[2]);

  Quaternion<double> qd;
  
  qd.coeffs() <<
    desired->att._value.qx,
    desired->att._value.qy,
    desired->att._value.qz,
    desired->att._value.qw;

  Matrix3d Rd_e, Rd_b;
  Rd_e = qd.matrix();
  Rd_b = Rd_e * ee_or.transpose();
  Quaternion<double> qb(Rd_b); 

  /* kinematics from end effector to body */
  Vector3d x_offset;
  x_offset = - (Rd_b * ee_pos);  
  desired->pos._value.x += x_offset(0);
  desired->pos._value.y += x_offset(1);
  desired->pos._value.z += x_offset(2);  

  desired->att._value.qx = qb.x();
  desired->att._value.qy = qb.y();
  desired->att._value.qz = qb.z();
  desired->att._value.qw = qb.w();

  Vector3d v_offset;
  Matrix3d s_w;
  s_w <<  0,                        -desired->avel._value.wz,  desired->avel._value.wy,
          desired->avel._value.wz,   0,                       -desired->avel._value.wx,
         -desired->avel._value.wy,   desired->avel._value.wx,  0;
  v_offset = - (s_w * Rd_b * ee_pos);   
  desired->vel._value.vx += v_offset(0);
  desired->vel._value.vy += v_offset(1);
  desired->vel._value.vz += v_offset(2); 

  Vector3d a_offset;
  Matrix3d s_dw;  
  s_dw <<  0,                        -desired->aacc._value.awz,  desired->aacc._value.awy,
          desired->aacc._value.awz,   0,                        -desired->aacc._value.awx,
         -desired->aacc._value.awy,   desired->aacc._value.awx,  0;

  a_offset = - (s_dw * Rd_b * ee_pos) - (s_w * s_w * Rd_b * ee_pos);
  desired->acc._value.ax += a_offset(0);
  desired->acc._value.ay += a_offset(1);
  desired->acc._value.az += a_offset(2);     
}