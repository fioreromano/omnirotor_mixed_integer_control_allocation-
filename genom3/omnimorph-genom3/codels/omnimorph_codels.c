/*
 * Copyright (c) 2015-2024 LAAS/CNRS
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
 *					Anthony Mallet on Tue Aug 11 2015
 *          		Youssef Aboudorra on Fri Jul 05 2024
 */
#include "acomnimorph.h"

#include <sys/time.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "omnimorph_c_types.h"

#include "codels.h"


/* --- Attribute set_saturation_weights --------------------------------- */

/** Validation codel omnimorph_set_satweights of attribute set_saturation_weights.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
omnimorph_set_satweights(const omnimorph_ids_servo_s_satweight_s *satweight,
                         const omnimorph_ids_body_s *body,
                         omnimorph_ids_servo_s *servo,
                         const genom_context self)
{
  omnimorph_controller_init(body, servo);
  return genom_ok;
}


/* --- Attribute set_wlimit --------------------------------------------- */

/** Validation codel omnimorph_set_wlimit of attribute set_wlimit.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
omnimorph_set_wlimit(double wmin, double wmax,
                     const omnimorph_ids_servo_s *servo,
                     omnimorph_ids_body_s *body,
                     const genom_context self)
{
  /* order bounds correctly */
  if (wmin > wmax) { double w = wmin; wmin = wmax; wmax = w; }

  omnimorph_wrench_bounds(body->G, wmin, wmax, body->wrench_min, body->wrench_max);
  omnimorph_controller_init(body, servo);
  return genom_ok;
}


/* --- Attribute set_mass ----------------------------------------------- */

/** Validation codel omnimorph_change_mass of attribute set_mass.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
omnimorph_change_mass(double mass, omnimorph_ids_body_s *body,
                      const genom_context self)
{
  /* update inertia matrix - this scales both the body mass and the rotors
   * mass. */

  if (mass <= 0.)
    return omnimorph_e_inval(&(omnimorph_e_inval_detail){"mass must be positive"}, self);

  if (isnan(body->mass)) return genom_ok;
  return omnimorph_scale_inertia(mass/body->mass, body->J, self);
}


/* --- Attribute set_geom ----------------------------------------------- */

/** Validation codel omnimorph_set_geom of attribute set_geom.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
omnimorph_set_geom(const double G[48], const double J[9],
                   omnimorph_ids_servo_s *servo,
                   omnimorph_ids_body_s *body,
                   const genom_context self)
{
  double w;
  int i, j;

  omnimorph_invert_J(J, body->iJ);
  omnimorph_invert_G(G, body->iG);
  omnimorph_wrench_bounds(
    G, body->wmin, body->wmax, body->wrench_min, body->wrench_max);

  /* count number of rotors from the iG matrix */
  for (i = or_rotorcraft_max_rotors - 1; i >= 0; i--) {
    w = 0.;
    for(j = 0; j < 6; j++) w += body->iG[i * 6 + j] * body->iG[i * 6 + j];
    if (w > 1e-6) break;
  }
  body->rotors = i + 1;
  body->init = true;
  servo->wmap_mode = omnimorph_modelled;

  omnimorph_controller_init(body, servo);
  return genom_ok;
}

/* --- Attribute set_wmaps ----------------------------------------------- */

/** Validation codel omnimorph_set_wmaps of attribute set_wmaps.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
omnimorph_set_wmaps(const double wmaps[480], const double J[9],
                   omnimorph_ids_servo_s *servo,
                   omnimorph_ids_body_s *body,
                   const genom_context self)
{
  double w;
  int i, j;
  //const double G[48];

  for (int i = 0; i<48; ++i){
    body->G[i] = wmaps[i];
  }

  const double *G = wmaps;

  omnimorph_invert_J(J, body->iJ);
  omnimorph_invert_G(G, body->iG);
  omnimorph_wrench_bounds(
    G, body->wmin, body->wmax, body->wrench_min, body->wrench_max);

  /* count number of rotors from the iG matrix */
  for (i = or_rotorcraft_max_rotors - 1; i >= 0; i--) {
    w = 0.;
    for(j = 0; j < 6; j++) w += body->iG[i * 6 + j] * body->iG[i * 6 + j];
    if (w > 1e-6) break;
  }
  body->rotors = i + 1;
  body->init = true;
  servo->wmap_mode = omnimorph_optimized;

  omnimorph_controller_init(body, servo);
  return genom_ok;
}

/* --- Attribute set_emerg ---------------------------------------------- */

/** Validation codel omnimorph_set_emerg of attribute set_emerg.
 *
 * Returns genom_ok.
 * Throws .
 */
genom_event
omnimorph_set_emerg(omnimorph_ids_servo_s_emerg_s *emerg,
                    const genom_context self)
{
  emerg->dx = emerg->dx * emerg->dx / 9.;
  emerg->dq = emerg->dq * emerg->dq / 9.;
  emerg->dv = emerg->dv * emerg->dv / 9.;
  emerg->dw = emerg->dw * emerg->dw / 9.;
  return genom_ok;
}


/* --- Activity servo --------------------------------------------------- */

/** Validation codel omnimorph_check_geom of activity servo.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_input, omnimorph_e_geom.
 */
genom_event
omnimorph_check_geom(bool init, const genom_context self)
{
  return init ? genom_ok : omnimorph_e_geom(self);
}


/* --- Function set_state ----------------------------------------------- */

/** Validation codel omnimorph_check_geom of function set_state.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
/* already defined in service servo validation */



/* --- Function set_position -------------------------------------------- */

/** Validation codel omnimorph_check_geom of function set_position.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
/* already defined in service servo validation */



/* --- Activity set_current_position ------------------------------------ */

/** Validation codel omnimorph_check_geom of activity set_current_position.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_input, omnimorph_e_geom.
 */
/* already defined in service servo validation */



/* --- Function set_velocity -------------------------------------------- */

/** Validation codel omnimorph_check_geom of function set_velocity.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
/* already defined in service servo validation */



/* --- Activity set_wo_zero --------------------------------------------- */

/** Validation codel omnimorph_check_geom of activity set_wo_zero.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
/* already defined in service servo validation */



/* --- Activity log ----------------------------------------------------- */

/** Validation codel omnimorph_log_open of activity log.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_sys.
 */
genom_event
omnimorph_log_open(const char path[64], uint32_t decimation,
                   omnimorph_log_s **log, const genom_context self)
{
  int fd;

  fd = open(path, O_WRONLY|O_APPEND|O_CREAT|O_TRUNC, 0666);
  if (fd < 0) return omnimorph_e_sys_error(path, self);

  if ((*log)->fd >= 0)
    close((*log)->fd);
  if ((*log)->req.aio_fildes >= 0) {
    if ((*log)->pending)
      while (aio_error(&(*log)->req) == EINPROGRESS)
        /* empty body */;
  }
  (*log)->fd = fd;
  (*log)->req.aio_fildes = -1;
  (*log)->pending = false;
  (*log)->skipped = false;
  (*log)->decimation = decimation < 1 ? 1 : decimation;
  (*log)->missed = 0;
  (*log)->total = 0;

  return genom_ok;
}


/* --- Function set_gtmrp_geom ------------------------------------------ */

/** Codel omnimorph_set_gtmrp_geom of function set_gtmrp_geom.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_inval.
 */
genom_event
omnimorph_set_gtmrp_geom(uint16_t rotors, double cx, double cy,
                         double cz, double armlen, double mass,
                         double mbodyw, double mbodyh, double mmotor,
                         double rx, double ry, int16_t rz, double cf,
                         double ct, const omnimorph_ids_servo_s *servo,
                         omnimorph_ids_body_s *body,
                         const genom_context self)
{
  genom_event e;

  e = omnimorph_gtmrp_allocmatrix(
    rotors, cx, cy, cz,
    armlen, rx * M_PI/180., ry * M_PI/180., rz, cf, ct, body->G, self);
  if (e) return e;

  e = omnimorph_set_geom(body->G, body->J, servo, body, self);
  if (e) return e;

  e = omnimorph_inertia(
    rotors, armlen, mass, mbodyw, mbodyh, mmotor, body->J, self);
  if (e) return e;

  body->mass = mass;
  body->rotors = rotors;
  body->init = true;

  omnimorph_controller_init(body, servo);
  return genom_ok;
}


/* --- Function get_reference ------------------------------------------- */

/** Codel omnimorph_get_reference of function get_reference.
 *
 * Returns genom_ok.
 */
genom_event
omnimorph_get_reference(const or_rigid_body_state *internal,
                        or_rigid_body_state *reference,
                        const genom_context self)
{
  *reference = *internal;

  /* explicitly use all fields to return data */
  reference->pos._present = true;
  reference->att._present = true;
  reference->vel._present = true;
  reference->avel._present = true;
  reference->acc._present = true;

  return genom_ok;
}


/* --- Function set_state ----------------------------------------------- */

/** Codel omnimorph_set_state of function set_state.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
genom_event
omnimorph_set_state(const or_t3d_pos *pos, const or_t3d_att *att,
                    const or_t3d_vel *vel, const or_t3d_avel *avel,
                    const or_t3d_acc *acc,
                    or_rigid_body_state *reference,
                    const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  if (isnan(pos->x))
    reference->pos._present = false;
  else {
    reference->pos._present = true;
    reference->pos._value = *pos;
  }

  if (isnan(att->qw))
    reference->att._present = false;
  else {
    reference->att._present = true;
    reference->att._value = *att;
  }

  if (isnan(vel->vx))
    reference->vel._present = false;
  else {
    reference->vel._present = true;
    reference->vel._value = *vel;
  }

  if (isnan(avel->wx))
    reference->avel._present = false;
  else {
    reference->avel._present = true;
    reference->avel._value = *avel;
  }

  if (isnan(acc->ax))
    reference->acc._present = false;
  else {
    reference->acc._present = true;
    reference->acc._value = *acc;
  }

  return genom_ok;
}


/* --- Function set_position -------------------------------------------- */

/** Codel omnimorph_set_position of function set_position.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
genom_event
omnimorph_set_position(double x, double y, double z, double yaw,
                       or_rigid_body_state *reference,
                       const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = true;
  reference->pos._value.x = x;
  reference->pos._value.y = y;
  reference->pos._value.z = z;

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

  return genom_ok;
}


/* --- Function set_velocity -------------------------------------------- */

/** Codel omnimorph_set_velocity of function set_velocity.
 *
 * Returns genom_ok.
 * Throws omnimorph_e_geom.
 */
genom_event
omnimorph_set_velocity(double x, double y, double z, double yaw,
                       or_rigid_body_state *reference,
                       const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = false;
  reference->att._present = false;

  reference->vel._present = true;
  reference->vel._value.vx = x;
  reference->vel._value.vy = y;
  reference->vel._value.vz = z;

  reference->avel._present = true;
  reference->avel._value.wx = 0.;
  reference->avel._value.wy = 0.;
  reference->avel._value.wz = yaw;

  reference->acc._present = false;
  return genom_ok;
}


/* --- Function stop ---------------------------------------------------- */

/** Codel omnimorph_servo_stop of function stop.
 *
 * Returns genom_ok.
 */
genom_event
omnimorph_servo_stop(or_rigid_body_state *reference,
                     const genom_context self)
{
  struct timeval tv;
  (void)self; /* -Wunused-parameter */

  gettimeofday(&tv, NULL);
  reference->ts.sec = tv.tv_sec;
  reference->ts.nsec = tv.tv_usec * 1000.;
  reference->intrinsic = false;

  reference->pos._present = true;
  reference->att._present = true;

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

  reference->aacc._present = false;
  reference->jerk._present = false;
  reference->snap._present = false;

  return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel omnimorph_log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
omnimorph_log_stop(omnimorph_log_s **log, const genom_context self)
{
  (void)self; /* -Wunused-parameter */

  if (*log && (*log)->req.aio_fildes >= 0)
    close((*log)->req.aio_fildes);
  (*log)->fd = (*log)->req.aio_fildes = -1;

  return genom_ok;
}


/* --- Function log_info ------------------------------------------------ */

/** Codel omnimorph_log_info of function log_info.
 *
 * Returns genom_ok.
 */
genom_event
omnimorph_log_info(const omnimorph_log_s *log, uint32_t *miss,
                   uint32_t *total, const genom_context self)
{
  *miss = *total = 0;
  if (log) {
    *miss = log->missed;
    *total = log->total;
  }
  return genom_ok;
}
