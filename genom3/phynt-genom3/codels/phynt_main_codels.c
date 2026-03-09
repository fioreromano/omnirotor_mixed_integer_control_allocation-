/*
 * Copyright (c) 2018,2023-2024 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 *                                      Anthony Mallet on Mon Jun 11 2018
 */
#include "acphynt.h"

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "phynt_c_types.h"
#include "codels.h"


/* --- Task main -------------------------------------------------------- */


/** Codel phynt_main_start of task main.
 *
 * Triggered by phynt_start.
 * Yields to phynt_init.
 */
genom_event
phynt_main_start(phynt_ids *ids, const phynt_desired *desired,
                 const genom_context self)
{
  ids->body = (phynt_ids_body_s){
    /* mikrokopter quadrotors defaults */
    .J = {
      0.015,    0.,    0.,
      0.,    0.015,    0.,
      0.,       0., 0.015
    },

    .mass = 1.0,
  };

  ids->af = (phynt_ids_af_s){
    .mass = ids->body.mass,
    .B = { 8., 8., 8., 8., 8., 8. },
    .K = { 10., 10., 10., 10., 10., 10. },
    .J =  {
      ids->body.J[0],             0.,             0.,
                  0., ids->body.J[4],             0.,
                  0.,             0., ids->body.J[8]
    },

    .force = { 0. },
    .torque = { 0. },
  };
  phynt_admittance_J(ids->af.J);

  ids->wo = (phynt_ids_wo_s){
    .K = { 10., 10., 10., 10., 10., 10. },
    .L = { 1., 1., 1., 1., 1., 1. },

    .fc = { 0. },
    .bias = { 0. },
    .thresh = { 0. },
  };

  ids->reference = (or_rigid_body_state){
    .ts = { .sec = 0, .nsec = 0 },
    .intrinsic = false,
    .pos._present = false,
    .att._present = false, .att._value = { .qw = 1. },
    .vel._present = false,
    .avel._present = false,
    .acc._present = false,
    .aacc._present = false,
    .jerk._present = false,
    .snap._present = false
  };

  ids->enable = (phynt_ids_enable_s){
    .wo = true,
    .af = true
  };

  if (desired->data(self)) {
    *desired->data(self) = ids->reference;
    desired->write(self);
  }


  /* init logging */
  ids->log = malloc(sizeof(*ids->log));
  if (!ids->log) abort();
  *ids->log = (phynt_log_s){
    .req = {
      .aio_fildes = -1,
      .aio_offset = 0,
      .aio_buf = ids->log->buffer,
      .aio_nbytes = 0,
      .aio_reqprio = 0,
      .aio_sigevent = { .sigev_notify = SIGEV_NONE },
      .aio_lio_opcode = LIO_NOP
    },
    .pending = false, .skipped = false,
    .decimation = 1, .missed = 0, .total = 0
  };

  return phynt_init;
}


/** Codel phynt_main_init of task main.
 *
 * Triggered by phynt_init.
 * Yields to phynt_pause_init, phynt_pause_control.
 */
genom_event
phynt_main_init(or_rigid_body_state *reference,
                const phynt_state *state, const genom_context self)
{
  const or_pose_estimator_state *state_data = NULL;

  /* switch to control mode upon reception of the first valid position or
   * velocity. Ensure that state has valid pos/att and initialize any empty
   * pos/att reference to it. */
  while (reference->pos._present || reference->vel._present) {
    if (state->read(self)) break;
    state_data = state->data(self);
    if (!state_data) break;
    if (!state_data->pos._present) break;
    if (!state_data->att._present) break;

    if (!reference->pos._present)
      reference->pos._value = state_data->pos._value;
    if (!reference->att._present)
      reference->att._value = state_data->att._value;

    return phynt_pause_control;
  }

  return phynt_pause_init;
}


/** Codel phynt_main_control of task main.
 *
 * Triggered by phynt_control.
 * Yields to phynt_pause_control.
 */
genom_event
phynt_main_control(const phynt_ids_body_s *body,
                   const phynt_ids_enable_s *enable,
                   phynt_ids_af_s *af, const phynt_ids_wo_s *wo,
                   const phynt_state *state,
                   const phynt_wrench_measure *wrench_measure,
                   or_rigid_body_state *reference, phynt_log_s **log,
                   const phynt_desired *desired,
                   const phynt_external_wrench *external_wrench,
                   const genom_context self)
{
  const or_pose_estimator_state *state_data = NULL;
  or_wrench_estimator_state *mwrench_data, *ewrench_data;
  or_rigid_body_state *desired_data;
  struct timeval tv;
  int s = 0;

  desired_data = desired->data(self);
  ewrench_data = external_wrench->data(self);
  if (!desired_data || !ewrench_data) return phynt_pause_control;

  gettimeofday(&tv, NULL);

  /* check reference */
  if (!reference->ts.sec) return phynt_pause_control;
  phynt_reference_check(tv, reference);

  /* current state */
  if (enable->wo || enable->af) {
    if (state->read(self) || !(state_data = state->data(self)))
      return phynt_pause_control;
    if (tv.tv_sec + 1e-6 * tv.tv_usec >
        0.5 + state_data->ts.sec + 1e-9 * state_data->ts.nsec)
      return phynt_pause_control;
  }
  
  /* measured wrench */
  if (enable->wo) {
    if (wrench_measure->read(self) ||
        !(mwrench_data = wrench_measure->data(self)))
      return phynt_pause_control;
  }

  /* external wrench */
  if (enable->wo) {
    s = phynt_wrench_observer(body, wo, state_data, mwrench_data, ewrench_data);
    if (!s) {
      ewrench_data->ts.sec = tv.tv_sec;
      ewrench_data->ts.nsec = tv.tv_usec * 1000;
    }
  } else {
    *ewrench_data = (or_wrench_estimator_state){
      .ts = { .sec = tv.tv_sec, .nsec = tv.tv_usec * 1000 },
      .force = {
        ._present = false,
        ._value = { nan(""), nan(""), nan("") }
      },
      .torque = {
        ._present = false,
        ._value = { nan(""), nan(""), nan("") }
      }
    };
  }
  external_wrench->write(self);

  /* admittance filter  */
  if (enable->af && !s)
    s = phynt_admittance_filter(body, af, reference, state_data, ewrench_data,
                                desired_data, *log);
  if (enable->af && !s) {
    desired_data->ts.sec = tv.tv_sec;
    desired_data->ts.nsec = tv.tv_usec * 1000;
  } else {
    *desired_data = *reference;
    phynt_kinematics_ee_to_body(body, state_data, desired_data);
  }
  desired->write(self);

  return phynt_pause_control;
}


/** Codel phynt_main_stop of task main.
 *
 * Triggered by phynt_stop.
 * Yields to phynt_ether.
 */
genom_event
phynt_main_stop(const phynt_desired *desired,
                const genom_context self)
{
  return phynt_ether;
}


/* --- Activity servo --------------------------------------------------- */

/** Codel phynt_servo_loop of activity servo.
 *
 * Triggered by phynt_start.
 * Yields to phynt_pause_start, phynt_ether.
 * Throws phynt_e_input.
 */
genom_event
phynt_servo_loop(const phynt_reference *in,
                 or_rigid_body_state *reference,
                 const genom_context self)
{
  const or_rigid_body_state *ref_data;

  if (in->read(self)) return phynt_e_input(self);
  ref_data = in->data(self);
  if (!ref_data) return phynt_e_input(self);

  /* check if timestamps have changed */
  if (reference->ts.nsec == ref_data->ts.nsec
      && reference->ts.sec == ref_data->ts.sec)
    return phynt_pause_start;

  /* keep old value of missing fields for phynt_reference_check */
#define copy_if_present(dst, src, f)                                    \
  do {                                                                  \
    dst->f._present = src->f._present;                                  \
    if (src->f._present) dst->f._value = src->f._value;                 \
  } while(0)

  copy_if_present(reference, ref_data, pos);
  copy_if_present(reference, ref_data, att);
  copy_if_present(reference, ref_data, vel);
  copy_if_present(reference, ref_data, avel);
  copy_if_present(reference, ref_data, acc);
  copy_if_present(reference, ref_data, aacc);
  copy_if_present(reference, ref_data, jerk);
  copy_if_present(reference, ref_data, snap);
#undef copy_if_present

  reference->ts = ref_data->ts;
  reference->intrinsic = ref_data->intrinsic;

  return phynt_pause_start;
}

/** Codel phynt_servo_stop of activity servo.
 *
 * Triggered by phynt_stop.
 * Yields to phynt_ether.
 * Throws phynt_e_input.
 */
genom_event
phynt_servo_stop(or_rigid_body_state *reference,
                 const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  reference->vel._present = false;
  reference->avel._present = false;
  reference->acc._present = false;
  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return phynt_ether;
}


/* --- Activity set_current_position ------------------------------------ */

/** Codel phynt_set_current_position of activity set_current_position.
 *
 * Triggered by phynt_start.
 * Yields to phynt_ether.
 * Throws phynt_e_input.
 */
genom_event
phynt_set_current_position(const phynt_state *state,
                           or_rigid_body_state *reference,
                           const genom_context self)
{
  const or_pose_estimator_state *state_data;
  double qw, qx, qy, qz;
  double yaw;

  if (state->read(self)) return phynt_e_input(self);
  state_data = state->data(self);
  if (!state_data) return phynt_e_input(self);
  if (!state_data->pos._present) return phynt_e_input(self);
  if (!state_data->att._present) return phynt_e_input(self);

  qw = state_data->att._value.qw;
  qx = state_data->att._value.qx;
  qy = state_data->att._value.qy;
  qz = state_data->att._value.qz;
  yaw = atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz));

  reference->ts = state_data->ts;

  reference->pos._present = true;
  reference->pos._value.x = state_data->pos._value.x;
  reference->pos._value.y = state_data->pos._value.y;
  reference->pos._value.z = state_data->pos._value.z;

  reference->att._present = true;
  reference->att._value.qw = cos(yaw/2.);
  reference->att._value.qx = 0.;
  reference->att._value.qy = 0.;
  reference->att._value.qz = sin(yaw/2.);

  reference->vel._present = true;
  reference->vel._value.vx = 0.;
  reference->vel._value.vy = 0.;
  reference->vel._value.vz = 0.;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = 0.;

  reference->acc._present = true;
  reference->acc._value.ax = 0.;
  reference->acc._value.ay = 0.;
  reference->acc._value.az = 0.;

  reference->jerk._present = false;
  reference->snap._present = false;

  return phynt_ether;
}


/* --- Activity set_wo_zero --------------------------------------------- */

/** Codel phynt_wo_zero_start of activity set_wo_zero.
 *
 * Triggered by phynt_start.
 * Yields to phynt_collect.
 */
genom_event
phynt_wo_zero_start(double accum[6], uint32_t *n,
                    const genom_context self)
{
  int i;

  for(i = 0; i < 6; i++) accum[i] = 0.;
  *n = 0;

  return phynt_collect;
}

/** Codel phynt_wo_zero_collect of activity set_wo_zero.
 *
 * Triggered by phynt_collect.
 * Yields to phynt_pause_collect, phynt_main.
 */
genom_event
phynt_wo_zero_collect(double duration,
                      const phynt_external_wrench *external_wrench,
                      double accum[6], uint32_t *n,
                      const genom_context self)
{
  or_wrench_estimator_state *wrench_data;

  wrench_data = external_wrench->data(self);

  if (wrench_data->force._present) {
    accum[0] += wrench_data->force._value.x;
    accum[1] += wrench_data->force._value.y;
    accum[2] += wrench_data->force._value.z;
  }
  if (wrench_data->torque._present) {
    accum[3] += wrench_data->torque._value.x;
    accum[4] += wrench_data->torque._value.y;
    accum[5] += wrench_data->torque._value.z;
  }

  return (++(*n) < duration * 1000./phynt_control_period_ms) ?
    phynt_pause_collect : phynt_main;
}

/** Codel phynt_wo_zero_main of activity set_wo_zero.
 *
 * Triggered by phynt_main.
 * Yields to phynt_ether.
 */
genom_event
phynt_wo_zero_main(const double accum[6], uint32_t n, double bias[6],
                   const genom_context self)
{
  int i;

  for(i = 0; i < 6; i++) bias[i] += accum[i] / n;

  return phynt_ether;
}
