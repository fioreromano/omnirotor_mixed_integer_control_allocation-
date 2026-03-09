/*
 * Copyright (c) 2016-2018,2021-2024 LAAS/CNRS
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
 *                                           Anthony Mallet on Tue Mar 22 2016
 */
#include "acomnimorph.h"

#include <aio.h>
#include <err.h>
#include <unistd.h>
#include <chrono>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>
#include <stack>
#include <limits>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/QR"
#include "proxsuite/proxqp/dense/dense.hpp"
#include "miqp_solver.cpp"

#include "codels.h"


/*
 * --- omnimorph_controller_init ------------------------------------------------
 *
 */

static proxsuite::proxqp::dense::QP<double> omnimorph_cont_alloc(8, 6, 8);
static proxsuite::proxqp::dense::QP<double> omnimorph_optimization_controller(14, 6, 14);
static proxsuite::proxqp::dense::QP<double> qp_fallback(8, 6, 0, true);
static miqp::MIQPSolver omnimorph_miqp_controller;
static double t_miqp, t_avg;
static int k_miqp, k_avg;
static bool first_call = true;
static int call_count = 0;
static int count_pinv = 0;
static int count_miqp = 0;
static int count_miqp_relaxed = 0;
static int count_qp = 0;
static int count_fallback_prev = 0;
static int count_fallback_miqp = 0;
static int count_fallback_miqp_relaxed = 0;
static long long time_pinv_total = 0;
static long long time_miqp_total = 0;
static long long time_miqp_relaxed_total = 0;
static long long time_qp_total = 0;
static long long time_fallback_prev_total = 0;
static long long time_fallback_miqp_total = 0;
static long long time_fallback_miqp_relaxed_total = 0;
static VectorXd y_prev = VectorXd::Ones(8);
static VectorXd u_prev = VectorXd::Constant(8, 50*50);
static VectorXd y_prev_measured = VectorXd::Ones(8);
static bool armed = false;
// Bounds costanti
const double lb = 30.0 * 30.0;
const double ub = 220.0 * 220.0;
const double lb_w = 30.0;
static const VectorXd lb_vec = VectorXd::Constant(8, lb);
static const VectorXd ub_vec = VectorXd::Constant(8, ub);
static const MatrixXd Q_identity = MatrixXd::Identity(8, 8);

void omnimorph_controller_init(const omnimorph_ids_body_s *body,
                          const omnimorph_ids_servo_s *servo)
{
  using namespace proxsuite::proxqp;
  using namespace Eigen;

  /* Controll (input) allocation optimization problem*/
  omnimorph_cont_alloc =
    dense::QP<double>(8, 6, 8, HessianType::Diagonal);
  omnimorph_cont_alloc.settings.verbose = false;
  omnimorph_cont_alloc.settings.compute_timings = true;
      
  /* TBD: describe the optimization problem*/
  const MatrixXd H = 2.* Matrix<double, Dynamic, Dynamic>::Identity(body->rotors, body->rotors);
  const VectorXd g = Matrix<double, Dynamic,1>::Zero(body->rotors);
  const MatrixXd C = Matrix<double, Dynamic, Dynamic>::Identity(body->rotors, body->rotors);
  /* lower/upper bounds for squared propellers velocities */
  VectorXd l(body->rotors), u(body->rotors);
  l.array() = body->wmin * std::fabs(body->wmin);
  u.array() = body->wmax * std::fabs(body->wmax);

  std::cout << "l:" << l << std::endl;
  std::cout << "prop: " << body->wmax * std::fabs(body->wmax) << std::endl;

  /* initialize with a fake problem first (average vertical thrust, no torque)*/
  const Map< const Matrix<double, Dynamic, 6, RowMajor> > iG(body->iG, body->rotors, 6);
  const Map< const Matrix<double, 6, Dynamic, RowMajor> > G(body->G, 6, body->rotors);
  std::cout << "G:" << G << std::endl;

  Matrix<double, 6, 1> b;
  b<< 0., 0., body->mass*9.81, 0., 0., 0. ;
  std::cout << "mass:" << body->mass << std::endl;
  std::cout << "b:" << b << std::endl;
  if(body->init){
    omnimorph_cont_alloc.init(H, g, G, b, C, l, u);
    omnimorph_cont_alloc.solve();
    if (omnimorph_cont_alloc.results.info.status == QPSolverOutput::PROXQP_SOLVED) 
    {
      std::cout << "control allocation solved" << std::endl;
      std::cout << "setup_time:" << omnimorph_cont_alloc.results.info.setup_time  << std::endl;
      std::cout << "solve_time:" << omnimorph_cont_alloc.results.info.solve_time  << std::endl;
      std::cout << "iter:" << omnimorph_cont_alloc.results.info.iter << std::endl;
      std::cout << "sol:" << omnimorph_cont_alloc.results.x << std::endl;  
    }
    else{
      std::cout << "control allocation not solved" << std::endl;
    }
  }
  

  /* Pose Controller Optimization problem */
  omnimorph_optimization_controller = dense::QP<double>(14, 6, 14, HessianType::Diagonal);
  omnimorph_optimization_controller.settings.verbose = false;
  omnimorph_optimization_controller.settings.compute_timings = true;

  /* TBD: describe the optimization problem*/

  const MatrixXd W1 = servo->weights.Wu * Matrix<double, 8, 8>::Identity(8, 8);
  //std::cout << "W1" << W1 << std::endl;

  Eigen::VectorXd W2_diag(6);
  W2_diag << servo->weights.Wx, servo->weights.Wy, servo->weights.Wz, 
              servo->weights.Wqx, servo->weights.Wqy, servo->weights.Wqz;
  const MatrixXd W2 = DiagonalMatrix<double, 6, 6> (W2_diag);
  //std::cout << "W2" << W2 << std::endl;

  const MatrixXd W3 = servo->weights.Wu_delta * Matrix<double, 8, 8>::Identity(8, 8);
  //std::cout << "W3" << W3 << std::endl;
  
  static Matrix<double, 6, 6> M;
  M.block(0,0,3,3) =  body->mass * Eigen::Matrix<double, 3, 3>::Identity(3,3);
  M.block(3,3,3,3) << Eigen::Matrix<double, 3, 3>(body->J);
  //std::cout << "M: "<< M << std::endl;
  
  static Matrix<double, 6, 1> mu;
  mu << 0., 0., body->mass * -9.81, 0., 0., 0.;
  Matrix<double, 6, 6> JR = Matrix<double, 6, 6>::Identity(6, 6);
  //std::cout << "JR: "<< JR << std::endl;


  static Matrix<double, 14, 14> Hessian;
  Hessian.block(0,0,6,6) = W2;
  Hessian.block(6,6,8,8) = W1;
  Hessian= 2*Hessian;
  //std::cout << "Hessian: " << std::endl << Hessian << std::endl;
  const VectorXd grad = Matrix<double, 14,1>::Zero(14);
  //std::cout << "grad: " << std::endl <<  grad << std::endl;
  static Matrix<double, 14, 14> Cin = Matrix<double, 14, 14>::Identity(14, 14);
  //std::cout << "Cin: " << std::endl << Cin << std::endl;
   
  qp_fallback.settings.eps_abs = 1e-4;
  qp_fallback.settings.eps_rel = 1e-4;
  qp_fallback.settings.max_iter = 10; 

  if (body->init)
  { 
  VectorXd l_in(14), u_in(14);
  l_in.segment(0,6) = -1000 * VectorXd::Ones(6,1);
  l_in.segment(6,8) = l;
  //std::cout << "l_in2: "<< std::endl << l_in << std::endl;

  u_in.segment(0,6) = 1000 * VectorXd::Ones(6);
  u_in.segment(6,8) = u;
  //std::cout << "u_in2: "<< std::endl << u_in << std::endl;

   static Matrix<double, 6, 14> Aqp;
  Aqp.block(0,0,6,6) = M;
  Aqp.block(0,6,6,8) = -JR*G;
  //std::cout << "Aqp: " << std::endl << Aqp << std::endl;

   omnimorph_optimization_controller.init(Hessian, grad, Aqp, mu, Cin, l_in, u_in);
    omnimorph_optimization_controller.solve();
    if (omnimorph_optimization_controller.results.info.status == QPSolverOutput::PROXQP_SOLVED) 
    {
      std::cout << "optimization control solved" << std::endl;
      //std::cout << "x: "<< omnimorph_optimization_controller.results.x << std::endl;
    }
    else{
      std::cout << "optimization control not solved" << std::endl;
    }
  }  
   
}


/*
 * --- omnimorph_controller -----------------------------------------------------
 *
 * Implements the controller described in:
 * TBD
 * 
 */

int omnimorph_controller(const omnimorph_ids_body_s *body,
                const omnimorph_ids_servo_s *servo,
                const or_pose_estimator_state *state,
                const or_rotorcraft_output *rotor_state,
                const or_rigid_body_state *reference,
                const or_wrench_estimator_state *mwrench,
                const or_wrench_estimator_state *ewrench,
                omnimorph_log_s *log,
                or_rotorcraft_rotor_control *wprop)
{
  using namespace Eigen;
  using namespace std::chrono;
  using namespace miqp;

  std::chrono::microseconds duration(0); 
  int method_used = 0;
  VectorXd u_best(body->rotors), y_best(body->rotors);

  Matrix3d Rd;
  Quaternion<double> qd;
  Vector3d xd, vd, wd, ad, awd, jd, awd_b;

  Matrix3d R;
  Quaternion<double> q;
  Vector3d x, v, w;

  Vector3d ex, ev, eR, ew;
  static Vector3d Iex;

  Vector3d f, m, wb;
  Vector3d f_ext, m_ext;

  Matrix<double, 6, 1> wrench;
  Matrix<double, 6, 1> ext_wrench;
  Matrix<double, 6, 1> ref_acc;

  Matrix<double, 6, 14> Aqp;
  Matrix<double, 14,1> grad;
  Matrix<double, 6, 6> M;
  M.block(0,0,3,3) =  body->mass * Eigen::Matrix<double, 3, 3>::Identity(3,3);
  M.block(3,3,3,3) << Eigen::Matrix<double, 3, 3>(body->J);
  Eigen::VectorXd W2_diag(6);
  W2_diag << servo->weights.Wx, servo->weights.Wy, servo->weights.Wz, 
              servo->weights.Wqx, servo->weights.Wqy, servo->weights.Wqz;
  MatrixXd W2 = DiagonalMatrix<double, 6, 6> (W2_diag);
  MatrixXd W3 = servo->weights.Wu_delta * Matrix<double, 8, 8>::Identity(8, 8);

  Matrix<double, 6, 1> mu;
  Matrix<double, 6, 6> JR = Matrix<double, 6, 6>::Identity(6, 6);
  Matrix<double, 14, 1> qp_res;
  static Matrix<double, 8, 1> u_old = Matrix<double, 8, 1>::Zero(8);

  Map< Matrix<double, or_rotorcraft_max_rotors, 1> > wprop_(wprop->_buffer);

  size_t i;

  /* geometry */
  const Map< const Matrix<double, or_rotorcraft_max_rotors, 6, RowMajor> > iG_(body->iG);
  const Map< const Matrix<double, 6, Dynamic, RowMajor> > G(body->G, 6, body->rotors);
  const Map< const Matrix3d > J(body->J); 
  const Map< const Matrix3d > iJ_(body->iJ);  

  /* gains */
  const Array3d Kp(servo->gain.Kpxy, servo->gain.Kpxy, servo->gain.Kpz);
  const Array3d Ki(servo->gain.Kixy, servo->gain.Kixy, servo->gain.Kiz);
  const Array3d Kv(servo->gain.Kvxy, servo->gain.Kvxy, servo->gain.Kvz);
  const Array3d Kq(servo->gain.Kqxy, servo->gain.Kqxy, servo->gain.Kqz);
  const Array3d Kw(servo->gain.Kwxy, servo->gain.Kwxy, servo->gain.Kwz);

  /* reference state - values are always valid due to omnimorph_reference_check() */
  xd <<
    reference->pos._value.x, reference->pos._value.y, reference->pos._value.z;
  qd.coeffs() <<
    reference->att._value.qx,
    reference->att._value.qy,
    reference->att._value.qz,
    reference->att._value.qw;
  vd <<
    reference->vel._value.vx,
    reference->vel._value.vy,
    reference->vel._value.vz;
  wd <<
    reference->avel._value.wx,
    reference->avel._value.wy,
    reference->avel._value.wz;
  ad <<
    reference->acc._value.ax,
    reference->acc._value.ay,
    reference->acc._value.az;
  awd <<
    reference->aacc._value.awx,
    reference->aacc._value.awy,
    reference->aacc._value.awz; 
  jd <<
    reference->jerk._value.jx,
    reference->jerk._value.jy,
    reference->jerk._value.jz;

  if (reference->intrinsic) {
    vd = qd * vd;
    wd = qd * wd;
    ad = qd * ad;
    awd = qd * awd;
    jd = qd * jd;
  }

  /* current state */
  if (state->pos._present) {
    x << state->pos._value.x, state->pos._value.y, state->pos._value.z;
  } else {
    x = xd;
    ad = Eigen::Vector3d(0, 0, - servo->emerg.descent);
    Iex << 0., 0., 0.;
  }

  if (state->att._present) {
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  } else {
    q = qd;
  }
  R = q.matrix();

  if (state->vel._present) {
    v << state->vel._value.vx, state->vel._value.vy, state->vel._value.vz;
  } else {
    v = vd;
  }

  if (state->avel._present) {
    w << state->avel._value.wx, state->avel._value.wy, state->avel._value.wz;
  } else {
    w = wd;
  }

  /* reference acceleration */
  //ad = Vector3d(0, 0, 9.81) + ad;

  /* external wrench */
  if(ewrench){
    if (ewrench->force._present) {
      f_ext << ewrench->force._value.x, ewrench->force._value.y, ewrench->force._value.z;
  } else {
    f_ext << 0., 0., 0.;
  }
  } else {
    f_ext << 0., 0., 0.;
  }

  if(ewrench){
    if (ewrench->torque._present) {
      m_ext << ewrench->torque._value.x, ewrench->torque._value.y, ewrench->torque._value.z; 
  } else {
    m_ext << 0., 0., 0.;
  }
  } else {
    m_ext << 0., 0., 0.;
  }

  /* external torque in body frame */
  if(ewrench){
    if (ewrench->torque._present){
      if (!(ewrench->intrinsic)){
        m_ext = R.transpose() * m_ext; 
      }
    } 
  }

  /* rotors speed measurement */
  VectorXd rot_meas(body->rotors);
  if (rotor_state) {
    for (int i = 0; i<body->rotors; i++){
      rot_meas(i) = rotor_state->rotor._buffer[i].velocity;      
    }
    if(!armed){
      for (int i = 0; i<body->rotors; i++){
          if(rot_meas(i) >= lb_w)
            armed = true;
          else{
            armed = false;
            break;
          }            
        }
      }
  }

  /* position error */
  ex = x - xd;
  for(i = 0; i < 3; i++)
    if (fabs(ex(i)) > servo->sat.x) ex(i) = copysign(servo->sat.x, ex(i));

  Iex += ex * omnimorph_control_period_ms/1000.;
  for(i = 0; i < 3; i++)
    if (fabs(Iex(i)) > servo->sat.ix) Iex(i) = copysign(servo->sat.ix, Iex(i));

  /* velocity error */
  ev = v - vd;
  for(i = 0; i < 3; i++)
    if (fabs(ev(i)) > servo->sat.v) ev(i) = copysign(servo->sat.v, ev(i));


     
  Rd = qd.matrix(); 

  /* orientation error */
  switch (servo->att_mode) {
    default: /* full attitude */ {
      Matrix3d E(0.5 * (Rd.transpose()*R - R.transpose()*Rd));

      eR <<
        (E(2, 1) - E(1, 2))/2.,
        (E(0, 2) - E(2, 0))/2.,
        (E(1, 0) - E(0, 1))/2.;
      break;
    }

    case omnimorph_tilt_prioritized:
      /* D. Brescianini and R. D’Andrea, “Tilt-Prioritized Quadrocopter
       * Attitude Control”, IEEE Transactions on Control Systems Technology,
       * vol. 28, no. 2, pp. 376–387, Mar. 2020. */

      /* attitude error in body frame s.t. q.qE = qd */
      Quaternion<double> qE(q.conjugate() * Quaternion<double>(Rd));

      /* reduced attitude and yaw error merged in a single vector */
      double n = hypot(qE.w(), qE.z());

      eR <<
        (qE.w() * qE.x() - qE.y() * qE.z()) / n,
        (qE.w() * qE.y() + qE.x() * qE.z()) / n,
        qE.z() / n;
      if (qE.w() < 0.) eR(2) = -eR(2);

      /* opposite and factor two to match sign and linearization of default
       * controller error so that gains are compatible */
      eR = - 2 * eR;
      break;
  }

  /* angular velocity error in body frame */
  if (reference->intrinsic){
    ew = R.transpose() * (w - Rd*wd);
  }
  else
    ew = R.transpose() * (w - wd);

  /* refrence angular acceleration in body frame */  
  /*if (reference->intrinsic){
    awd_b = awd;
  }
  else
    awd_b = Rd.transpose() * awd;*/
  switch (servo->control_mode){
    default:
      //f = body->mass * ((- Kp * ex.array() - Kv * ev.array() - Ki * Iex.array()).matrix() + Eigen::Vector3d(0, 0, 9.81) + ad - (1/body->mass) * f_ext);
      f = body->mass * ((- Kp * ex.array() - Kv * ev.array() - Ki * Iex.array()).matrix() + Eigen::Vector3d(0, 0, 9.81) + ad);
      wb = q.inverse() * w;
      //m = (wb.cross(J * wb)) - m_ext + J * ((- Kq * eR.array() - Kw * ew.array()).matrix() + qd.inverse()*awd);
      m = (wb.cross(J * wb)) + J * ((- Kq * eR.array() - Kw * ew.array()).matrix() + qd.inverse()*awd);
      //m = J * ((- Kq * eR.array() - Kw * ew.array()).matrix()) - m_ext;

      /* wrench in body frame */
      wrench.block<3, 1>(0, 0) = q.inverse() * f;
      wrench.block<3, 1>(3, 0) = m;

      /* wrench saturation: ensure wmin² <= G¯¹.wrench <= wmax² */
      wprop->_length = body->rotors;
      wprop_.noalias() = iG_ * wrench;

      using namespace proxsuite::proxqp;
      omnimorph_cont_alloc.update(
      proxsuite::nullopt, proxsuite::nullopt,
      G, wrench,
      proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt);
      omnimorph_cont_alloc.solve();
      if (omnimorph_cont_alloc.results.info.status == QPSolverOutput::PROXQP_SOLVED) 
      {
        wprop_.head(body->rotors).noalias() = omnimorph_cont_alloc.results.x;
      }
      else{
        std::cout << " Controll allocation not solved " << std::endl;
      }
      break;
     
    case omnimorph_optimization:
      ref_acc.block<3,1>(0, 0) = - Kp * ex.array() - Kv * ev.array() - Ki * Iex.array();
      ref_acc.block<3,1>(3, 0) = - Kq * eR.array() - Kw * ew.array();
      JR.block(0,0,3,3) = R;
      JR.block(0,3,3,3) = Eigen::Matrix<double, 3, 3>::Identity(3, 3);
      Aqp.block(0,0,6,6) = M;
      Aqp.block(0,6,6,8) = -JR*G;
      grad.segment(0,6) = -2*W2*ref_acc;
      grad.segment(6,8) = -2*W3*u_old;
      using namespace proxsuite::proxqp;
      omnimorph_optimization_controller.update(
      proxsuite::nullopt, grad, Aqp, proxsuite::nullopt,
      proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt);
      omnimorph_optimization_controller.solve();
      if (omnimorph_optimization_controller.results.info.status == QPSolverOutput::PROXQP_SOLVED){
        qp_res = omnimorph_optimization_controller.results.x;
        wprop_.head(body->rotors).noalias() = qp_res.segment(6,8);
        u_old = qp_res.segment(6,8);
      }
      else{
        std::cout << " Optimization control problem not solved " << std::endl;
      }
      break;
      
    case omnimorph_miqp: 
      auto start_time = std::chrono::high_resolution_clock::now();
      double previous_time = 0.0;

      // --- 1. DYNAMICS & ALLOCATION MATRIX ---
      ref_acc.block<3,1>(0,0) =
          (-Kp * ex.array() - Kv * ev.array() - Ki * Iex.array()).matrix()
          + ad;

      // rotational virtual input (body frame: eR, ew are body)
      ref_acc.block<3,1>(3,0) =
          (-Kq * eR.array() - Kw * ew.array()).matrix()
          + (qd.inverse() * awd);
      
      Vector3d omega_b = q.inverse() * w;
      Matrix3d J_inv = J.inverse();
      Matrix<double, 6, 6> invInertia_mat = Matrix<double, 6, 6>::Zero();
      invInertia_mat.block<3,3>(0,0) = Matrix3d::Identity() * (1/body->mass);
      invInertia_mat.block<3,3>(3,3) = J_inv;
      // Jacobiano Rotazione (JR)
      JR.block<3,3>(0,0) = R;
      JR.block<3,3>(3,3) = Matrix3d::Identity();
      // Allocation Matrix A_bar
      MatrixXd A_bar = invInertia_mat * JR * G;
      // Bias Forces (Coriolis + Gravity)
      VectorXd bias_forces(6);
      bias_forces.head(3) = -Eigen::Vector3d(0, 0, 9.81);
      bias_forces.tail(3) = -J_inv * (omega_b.cross(J * omega_b));
      // External wrench
      ext_wrench.block<3,1>(0, 0) = f_ext;
      ext_wrench.block<3,1>(3, 0) = m_ext;
      // Feedback linearization
      VectorXd a_miqp = - bias_forces + ref_acc - invInertia_mat*ext_wrench;   

      double cost_best;   bool pinv_feasible = true;   
      // --- 3. MEASUREMENT UPDATE ---
      VectorXd y_measured(body->rotors);      
      for(int i = 0; i < 8; i++) {
          if(std::abs(rot_meas(i)) < 1e-6) {
              y_measured(i) = y_prev_measured(i); 
          } else {
              y_measured(i) = (rot_meas(i) > 1e-6) ? 1.0 : 0.0;
          }
      }
      y_prev_measured = y_measured;
      VectorXd u_pinv = A_bar.completeOrthogonalDecomposition().solve(a_miqp);
      for(int i = 0; i < 8; i++) {
          if(std::abs(u_pinv(i)) < lb || std::abs(u_pinv(i)) > ub) {
              pinv_feasible = false;
              break;
          }
      }      
      std::vector<int> near_lb_idx;
      for(int i = 0; i < 8; i++) {
          if(y_prev(i) != y_prev_measured(i)) {
              near_lb_idx.push_back(i);
          }         
      }      
      bool all_measu = true;
      if(armed) {
        for(int i = 0; i < 8; i++) {
          if(!(y_prev(i) == y_prev_measured(i) && std::abs(rot_meas(i)) >= lb_w)) {
              all_measu = false;
              break;
          }
        }
      }                  
       
      VectorXd prop_speed_sq(8);
      MIQPResult res;

      /*
      METHOD USED : 
      - 1 -> PSEUDOINVERSE
      - 2 -> MIQP
      - 3 -> RELAXED QP
      - 4 -> QP WITH EQUALITY
      - 5 -> FALLBACK MIQP TIMEOUT
      - 6 -> FALLBACK QP RELAXED TIMEOUT
      - 7 -> FALLBACK PSEUDOINVERSE
      */
      
      if(pinv_feasible && all_measu) {
          // Using the Pseudoinverse solution directly
          res.u_best = u_pinv;
          res.y_best = (u_pinv.array() > 0).cast<double>();
          res.flag = true;
          method_used = 1;
      } 
      else {
          miqp::Objective objective;
          objective.Q = Q_identity;
          objective.f = -u_prev; 
          bool Aeq_on = true;
          VectorXd y_fixed = VectorXd::Constant(8, std::numeric_limits<double>::quiet_NaN());              
          if(!near_lb_idx.empty()) { // propeller near lower bound, solve full MIQP with y fixed
            for(int idx : near_lb_idx) {
                y_fixed(idx) = y_prev(idx);
            }  
          } 
          std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start_time;
          previous_time = elapsed.count();          
          res = omnimorph_miqp_controller.solveMIQP(a_miqp, A_bar, objective, 
                                          VectorXd::Constant(8, lb), 
                                          VectorXd::Constant(8, ub),
                                          u_prev, y_prev, y_fixed, Aeq_on, previous_time);   
          method_used = 2;       
          if (res.status == miqp::MIQPSolverStatus::TIMEOUT) {
            res.u_best = u_prev;
            res.y_best = (u_prev.array() > 0).cast<double>();
            method_used = 5;
          }
          else if(res.status == miqp::MIQPSolverStatus::INFEASIBLE || res.status == miqp::MIQPSolverStatus::ERROR) {
            // First fallback: relaxed objective if infeasible
            miqp::Objective objective;
            double rho = 1e2;
            objective.Q = Q_identity + rho*A_bar.transpose() * A_bar;
            objective.f = -rho*A_bar.transpose() * a_miqp - u_prev;          
            bool Aeq_on = false;              // remove equality constraint              
            VectorXd y_fixed = VectorXd::Constant(8, std::numeric_limits<double>::quiet_NaN());
            previous_time += res.solve_time_ms;
            res = omnimorph_miqp_controller.solveMIQP(a_miqp, A_bar, objective,
                                            VectorXd::Constant(8, lb),
                                            VectorXd::Constant(8, ub),
                                            u_prev, y_prev, y_fixed, Aeq_on, previous_time); 
            method_used =  3;   
            if (res.status == miqp::MIQPSolverStatus::TIMEOUT) {
              res.u_best = u_prev;
              res.y_best = (u_prev.array() > 0).cast<double>();
              method_used = 6;
            }          // Second fallback: plain QP with equality
            else if(res.status == miqp::MIQPSolverStatus::INFEASIBLE || res.status == miqp::MIQPSolverStatus::ERROR) {
                qp_fallback.update(Q_identity, -u_prev, A_bar, a_miqp, proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, -ub_vec, ub_vec); 
                qp_fallback.results.x = u_prev; // warm start 
                qp_fallback.results.y.setZero();
                qp_fallback.results.z.setZero();

                qp_fallback.solve(); 

                if(qp_fallback.results.info.status == proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) { 
                  res.u_best = qp_fallback.results.x.head(8); 
                  res.y_best = (res.u_best.array() > 0).cast<double>();
                  res.flag = true; 
                  method_used = 4;
                }
                else {
                  // QP also infeasible, using pinv u.
                  res.u_best = u_pinv;
                  res.y_best = (u_pinv.array() > 0).cast<double>();
                  res.flag = false; 
                  method_used = 7;
                }
            }    
          } 
        }                             
      // Store final results
      u_best = res.u_best;
      y_best = res.y_best;
      u_prev = u_best;
      y_prev = y_best;                
      // Output final rotor speeds
      prop_speed_sq = u_best;
      wprop_.head(body->rotors).noalias() = prop_speed_sq;
      auto end_time = std::chrono::high_resolution_clock::now();
      duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      t_miqp += duration.count(); 
      k_miqp++;
      if (method_used == 1) {
        count_pinv++;
        time_pinv_total += duration.count();
      }
      else if (method_used == 2) {
        count_miqp++;
        time_miqp_total += duration.count();
      }
      else if (method_used == 3) {
        count_miqp_relaxed++;
        time_miqp_relaxed_total += duration.count();
      }
      else if (method_used == 4) {
         count_qp++;
         time_qp_total += duration.count();
      }
      else if (method_used == 7) {
        count_fallback_prev++;
        time_fallback_prev_total += duration.count();
      }
      else if (method_used == 5) {
        count_fallback_miqp++;
        time_fallback_miqp_total += duration.count();
      }
      else if (method_used == 6) {
        count_fallback_miqp_relaxed++;
        time_fallback_miqp_relaxed_total += duration.count();
      }
      call_count++;
      //std::cout << "Control Execution time: "<< duration.count() << std::endl;
      break; 
  } 
  /* output: signed square root and extra clipping to really enforce limits in
   * case the saturation is disabled did not find a solution */
  wprop_.head(body->rotors) = wprop_.head(body->rotors).unaryExpr([body](double w2) {
      double w = std::copysign(std::sqrt(std::fabs(w2)), w2);
      return w < body->wmin ? body->wmin : w > body->wmax ? body->wmax : w;
    });  

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
      double yaw = atan2(Rd(1,0), Rd(0,0));
      double pitch = asin(-Rd(2,0));
      double roll = atan2(Rd(2,1), Rd(2,2));
      log->req.aio_nbytes = snprintf(
        log->buffer, sizeof(log->buffer),
        "%s " omnimorph_log_fmt "\n",
        log->skipped ? "\n" : "",
        state->ts.sec, state->ts.nsec,
        state->ts.sec - reference->ts.sec +
        (state->ts.nsec - reference->ts.nsec)*1e-9,
        wrench(0), wrench(1), wrench(2), wrench(3), wrench(4), wrench(5),
        mwrench->force._present ? mwrench->force._value.x : nan(""),
        mwrench->force._present ? mwrench->force._value.y : nan(""),
        mwrench->force._present ? mwrench->force._value.z : nan(""),
        mwrench->torque._present ? mwrench->torque._value.x : nan(""),
        mwrench->torque._present ? mwrench->torque._value.y : nan(""),
        mwrench->torque._present ? mwrench->torque._value.z : nan(""),
        xd(0), xd(1), xd(2), roll, pitch, yaw,
        vd(0), vd(1), vd(2), wd(0), wd(1), wd(2),
        ad(0), ad(1), ad(2),
        ex(0), ex(1), ex(2), ev(0), ev(1), ev(2),
        eR(0), eR(1), eR(2), ew(0), ew(1), ew(2),
        f(0), f(1), f(2),
        omnimorph_cont_alloc.results.x(0),
        omnimorph_cont_alloc.results.x(1),
        omnimorph_cont_alloc.results.x(2),
        omnimorph_cont_alloc.results.x(3),
        omnimorph_cont_alloc.results.x(4),
        omnimorph_cont_alloc.results.x(5),
        omnimorph_cont_alloc.results.x(6),
        omnimorph_cont_alloc.results.x(7),
        wprop_(0), wprop_(1), wprop_(2), wprop_(3), wprop_(4), wprop_(5), wprop_(6), wprop_(7),
        omnimorph_cont_alloc.results.info.setup_time,
        omnimorph_cont_alloc.results.info.solve_time,
        omnimorph_cont_alloc.results.info.run_time,
        ref_acc(0), ref_acc(1), ref_acc(2), ref_acc(3), ref_acc(4), ref_acc(5),
        omnimorph_optimization_controller.results.x(0),
        omnimorph_optimization_controller.results.x(1),
        omnimorph_optimization_controller.results.x(2),
        omnimorph_optimization_controller.results.x(3),
        omnimorph_optimization_controller.results.x(4),
        omnimorph_optimization_controller.results.x(5),
        omnimorph_optimization_controller.results.x(6),
        omnimorph_optimization_controller.results.x(7),
        omnimorph_optimization_controller.results.x(8),
        omnimorph_optimization_controller.results.x(9),
        omnimorph_optimization_controller.results.x(10),
        omnimorph_optimization_controller.results.x(11),
        omnimorph_optimization_controller.results.x(12),
        omnimorph_optimization_controller.results.x(13),
        omnimorph_optimization_controller.results.info.setup_time,
        omnimorph_optimization_controller.results.info.solve_time,
        omnimorph_optimization_controller.results.info.run_time,
        method_used,
        duration.count(),
        y_best(0), y_best(1), y_best(2), y_best(3), y_best(4), y_best(5), y_best(6), y_best(7),
        call_count, count_pinv, count_miqp, count_miqp_relaxed, count_qp, 
        count_fallback_prev, count_fallback_miqp, count_fallback_miqp_relaxed,
        (count_pinv > 0 ? (time_pinv_total / count_pinv) : 0.0),
        (count_miqp > 0 ? (time_miqp_total / count_miqp) : 0.0),
        (count_miqp_relaxed > 0 ? (time_miqp_relaxed_total / count_miqp_relaxed) : 0.0),
        (count_qp > 0 ? (time_qp_total / count_qp) : 0.0),
        (count_fallback_prev > 0 ? (time_fallback_prev_total / count_fallback_prev) : 0.0),
        (count_fallback_miqp > 0 ? (time_fallback_miqp_total / count_fallback_miqp) : 0.0),
        (count_fallback_miqp_relaxed > 0 ? (time_fallback_miqp_relaxed_total / count_fallback_miqp_relaxed) : 0.0)
        );
      if (aio_write(&log->req)) {
        warn("log");
        close(log->req.aio_fildes);
        log->req.aio_fildes = -1;
      } else
        log->pending = true;

      log->skipped = false;
    }
  }
  return 0;
}

/*
 * --- omnimorph_state_check ----------------------------------------------------
 *
 * Return non-zero in case of emergency
 */

int omnimorph_state_check(const struct timeval now,
                 const omnimorph_ids_servo_s *servo,
                 or_pose_estimator_state *state)
{
  int e = omnimorph_EOK;

  /* check state */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      0.5 + state->ts.sec + 1e-9 * state->ts.nsec) {
    state->pos._present = false;
    state->att._present = false;
    state->vel._present = false;
    state->avel._present = false;
    return omnimorph_ETS;
  }

  if (!state->pos._present
      || std::isnan(state->pos._value.x)
      || !state->pos_cov._present
      || state->pos_cov._value.cov[0] > servo->emerg.dx
      || state->pos_cov._value.cov[2] > servo->emerg.dx
      || state->pos_cov._value.cov[5] > servo->emerg.dx) {
    state->pos._present = false;
    e |= omnimorph_EPOS;
  }

  if (!state->att._present
      || std::isnan(state->att._value.qw) ||
      !state->att_cov._present ||
      state->att_cov._value.cov[0] > servo->emerg.dq ||
      state->att_cov._value.cov[2] > servo->emerg.dq ||
      state->att_cov._value.cov[5] > servo->emerg.dq ||
      state->att_cov._value.cov[9] > servo->emerg.dq) {
    state->att._present = false;
    e |= omnimorph_EATT;
  }

  if (!state->vel._present
      || std::isnan(state->vel._value.vx)
      || !state->vel_cov._present
      || state->vel_cov._value.cov[0] > servo->emerg.dv
      || state->vel_cov._value.cov[2] > servo->emerg.dv
      || state->vel_cov._value.cov[5] > servo->emerg.dv) {
    state->vel._present = false;
    e |= omnimorph_EVEL;
  }

  if (!state->avel._present
      || std::isnan(state->avel._value.wx)
      || !state->avel_cov._present
      || state->avel_cov._value.cov[0] > servo->emerg.dw
      || state->avel_cov._value.cov[2] > servo->emerg.dw
      || state->avel_cov._value.cov[5] > servo->emerg.dw) {
    state->avel._present = false;
    e |= omnimorph_EAVEL;
  }

  return e;
}


/*
 * --- omnimorph_reference_check ------------------------------------------------
 *
 * Update missing fields of the reference by integrating other fields
 */

void
omnimorph_reference_check(const struct timeval now,
                     or_rigid_body_state *reference)
{
  static const double dt = omnimorph_control_period_ms / 1000.;
  static const double dt2 = dt * dt;
  static const double dt2_2 = dt2 / 2.;

  or_t3d_pos *p = &reference->pos._value;
  or_t3d_att *q = &reference->att._value;
  or_t3d_vel *v = &reference->vel._value;
  or_t3d_avel *w = &reference->avel._value;
  or_t3d_acc *a = &reference->acc._value;
  or_t3d_aacc *aw = &reference->aacc._value;
  or_t3d_jerk *j = &reference->jerk._value;

  /* deal with obsolete reference */
  if (now.tv_sec + 1e-6 * now.tv_usec >
      0.5 + reference->ts.sec + 1e-9 * reference->ts.nsec) {
    reference->vel._present = true;
    v->vx = v->vy = v->vz = 0.;

    reference->avel._present = true;
    w->wx = w->wy = w->wz = 0.;

    reference->acc._present = true;
    a->ax = a->ay = a->az = 0.;

    reference->jerk._present = true;
    j->jx = j->jy = j->jz = 0.;
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

  if (!reference->jerk._present) {
    /* reset */
    j->jx = j->jy = j->jz = 0.;
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
 * --- omnimorph_wrench ---------------------------------------------------------
 *
 * Compute measured total wrench
 */

int
omnimorph_wrench(const omnimorph_ids_body_s *body,
            const or_pose_estimator_state *state,
            const double wprop[or_rotorcraft_max_rotors],
            double wrench[6])
{
  using namespace Eigen;

  Quaternion<double> q;
  Map< const Array<double, or_rotorcraft_max_rotors, 1> >wprop_(wprop);
  Map< Matrix<double, 6, 1> >wrench_(wrench);

  Map< const Matrix<double, 6, or_rotorcraft_max_rotors, RowMajor> > G(body->G);

  /* current state - XXX do something if state not present / uncertain */
  if (state->att._present && !std::isnan(state->att._value.qw))
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  else 
    q = Quaternion<double>::Identity();

  wrench_ = G * (wprop_.sign() * wprop_.square()).matrix();
  wrench_.block<3, 1>(0, 0) = q * wrench_.block<3, 1>(0, 0);
  wrench_.block<3, 1>(3, 0) = q * wrench_.block<3, 1>(3, 0);

  return 0;
}

/*
 * --- omnimorph_select_wmap ----------------------------------------------------
 *
 * 
 */

void
omnimorph_select_wmap(omnimorph_ids_body_s *body,
                    const omnimorph_ids_servo_s *servo,
                    const or_pose_estimator_state *state)
{
  using namespace Eigen;

  Quaternion<double>  q;
  if (state->att._present && !std::isnan(state->att._value.qw))
    q.coeffs() <<
      state->att._value.qx, state->att._value.qy, state->att._value.qz,
      state->att._value.qw;
  else 
    q = Quaternion<double>::Identity();

  Matrix3d R = q.matrix();
  Vector3d vec(0.0, 0.0, 1.0);
  Vector3d rotated = R.transpose() * vec;

  double azimuth = std::atan2(rotated.y(), rotated.x()); // θ: in XY-plane from X-axis
  double elevation = std::atan2(rotated.z(), std::sqrt(rotated.x() * rotated.x() + rotated.y() * rotated.y())); // φ: from horizontal plane

  Map<Matrix<double, 6, Dynamic, RowMajor> > G(body->G, 6, body->rotors);

  switch (servo->wmap_mode){
    default:     
      //body->G = G;
      break;
    case omnimorph_optimized:
      // Define the 10 regions
      struct Region {
        double az_min, az_max;
        double az2_min, az2_max;
        double el_min, el_max;
      };

      constexpr double pi = M_PI;

      std::vector<Region> regions = {
        { -pi,      pi,     -pi,      pi,      pi/3,    pi/2   },  // Region 0
        { -pi/4,    pi/4,   -pi/4,    pi/4,    0,       pi/3   },  // Region 1
        { -3*pi/4, -pi/4,   -3*pi/4, -pi/4,    0,       pi/3   },  // Region 2
        {  pi/4,    3*pi/4,  pi/4,    3*pi/4,  0,       pi/3   },  // Region 3
        { -pi,     -3*pi/4,  3*pi/4,  pi,      0,       pi/3   },  // Region 4
        { -pi,      pi,     -pi,      pi,     -pi/2,   -pi/3   },  // Region 5
        { -pi/4,    pi/4,   -pi/4,    pi/4,   -pi/3,    0      },  // Region 6
        { -3*pi/4, -pi/4,   -3*pi/4, -pi/4,   -pi/3,    0      },  // Region 7
        {  pi/4,    3*pi/4,  pi/4,    3*pi/4, -pi/3,    0      },  // Region 8
        { -pi,     -3*pi/4,  3*pi/4,  pi,     -pi/3,    0      },  // Region 9
      };      
      
      for (int i = 0; i < regions.size(); ++i) {
        const Region& reg = regions[i];
        bool az_in_range = (azimuth >= reg.az_min && azimuth <= reg.az_max) ||
                           (azimuth >= reg.az2_min && azimuth <= reg.az2_max);
        bool el_in_range = (elevation >= reg.el_min && elevation <= reg.el_max);

        if (az_in_range && el_in_range) {
          Map<const Matrix<double, 6, 8, RowMajor>> wmap(&body->wmaps[48 * i]);
          G = wmap;
          //std::cout << "Region: " << i << std::endl;
          break;
        }
      }            
      break;
  }

}

