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
#ifndef H_omnimorph_CODELS
#define H_omnimorph_CODELS

#include <aio.h>

#include "omnimorph_c_types.h"

enum omnimorphe {
  omnimorph_EOK =	0,
  omnimorph_ETS =	1 << 0,
  omnimorph_EPOS =	1 << 1,
  omnimorph_EATT =	1 << 2,
  omnimorph_EVEL =	1 << 3,
  omnimorph_EAVEL =	1 << 4,
};

#ifdef __cplusplus
extern "C" {
#endif

  void  omnimorph_select_wmap(omnimorph_ids_body_s *body,
                const omnimorph_ids_servo_s *servo,
                const or_pose_estimator_state *state);
  void	omnimorph_controller_init(const omnimorph_ids_body_s *body,
                const omnimorph_ids_servo_s *servo);
  int	omnimorph_controller(const omnimorph_ids_body_s *body,
                const omnimorph_ids_servo_s *servo,
                const or_pose_estimator_state *state,
                const or_rotorcraft_output *rotor_state,
                const or_rigid_body_state *desired,
                const or_wrench_estimator_state *exwrench,
                const or_wrench_estimator_state *ewrench,
                omnimorph_log_s *log,
                or_rotorcraft_rotor_control *wprop);
  int	omnimorph_state_check(const struct timeval now,
                const omnimorph_ids_servo_s *servo,
                or_pose_estimator_state *state);
  void	omnimorph_reference_check(const struct timeval now,
                or_rigid_body_state *reference);
  int	omnimorph_wrench(const omnimorph_ids_body_s *body,
                const or_pose_estimator_state *state,
                const double wprop[or_rotorcraft_max_rotors],
                double wrench[6]);

  genom_event
	omnimorph_gtmrp_allocmatrix(int rotors, double cx, double cy, double cz,
                double armlen, double rx, double ry, double rz, double cf,
                double ct, double G[6 * or_rotorcraft_max_rotors],
                const genom_context self);
  genom_event
	omnimorph_inertia(int rotors, double armlen, double mass,
                double mbodyw, double mbodyh, double mmotor, double J[3 * 3],
                const genom_context self);
  genom_event
	omnimorph_scale_inertia(double s, double J[3 * 3], const genom_context self);

  void	omnimorph_invert_G(const double G[6 * or_rotorcraft_max_rotors],
                double iG[or_rotorcraft_max_rotors * 6]);
  void	omnimorph_invert_J(const double J[3 * 3], double iJ[3 * 3]);                
  void	omnimorph_wrench_bounds(const double G[6 * or_rotorcraft_max_rotors],
                const double wmin, const double wmax, double fmin[6],
                double fmax[6]);

#ifdef __cplusplus
}
#endif

static inline genom_event
omnimorph_e_sys_error(const char *s, genom_context self)
{
  omnimorph_e_sys_detail d;
  size_t l = 0;

  d.code = errno;
  if (s) {
    strncpy(d.what, s, sizeof(d.what) - 3);
    l = strlen(s);
    strcpy(d.what + l, ": ");
    l += 2;
  }
  if (strerror_r(d.code, d.what + l, sizeof(d.what) - l)) { /* ignore error*/; }
  return omnimorph_e_sys(&d, self);
}

struct omnimorph_log_s {
  int fd;
  struct aiocb req;
  char buffer[4096];
  bool pending, skipped;
  uint32_t decimation;
  size_t missed, total;

# define omnimorph_logfmt	" %g "
# define omnimorph_logfmt_int	" %d "
# define omnimorph_log_header_fmt                                            \
  "ts delay "                                                           \
  "fx fy fz tx ty tz "                                                  \
  "meas_fx meas_fy meas_fz meas_tx meas_ty meas_tz "                    \
  "xd yd zd rolld pitchd yawd vxd vyd vzd wxd wyd wzd axd ayd azd "     \
  "e_x e_y e_z e_vx e_vy e_vz e_rx e_ry e_rz e_wx e_wy e_wz "           \
  "fwx fwy fwz "                                                        \
  "w2_1 w2_2 w2_3 w2_4 w2_5 w2_6 w2_7 w2_8 "                            \
  "w1 w2 w3 w4 w5 w6 w7 w8 "                                            \
  "setup_time sol_time run_time "                                       \
  "refacc_x refacc_y refacc_z refacc_qx refacc_qy refacc_qz "           \
  "q1 q2 q3 q4 q5 q6 "                                                  \
  "wc2_1 wc2_2 wc2_3 wc2_4 wc2_5 wc2_6 wc2_7 wc2_8 "                    \
  "setup_time_c sol_time_c run_time_c "                                 \
  "method_used solve_time_us "                                          \
  "ybest_1 ybest_2 ybest_3 ybest_4 ybest_5 ybest_6 ybest_7 ybest_8 "    \
  "call_count count_pinv count_miqp count_miqp_relaxed count_qp "       \
  "count_fallback_prev count_fallback_miqp count_fallback_miqp_relaxed " \
  "avg_time_pinv avg_time_miqp avg_time_miqp_relaxed avg_time_qp " 	   \
  "avg_time_fallback_prev avg_time_fallback_miqp avg_time_fallback_miqp_relaxed "
# define omnimorph_log_fmt                                                   \
  "%d.%09d %g "                                                           \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt_int "%ld " \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt_int omnimorph_logfmt_int omnimorph_logfmt_int omnimorph_logfmt_int omnimorph_logfmt_int \
  omnimorph_logfmt_int omnimorph_logfmt_int omnimorph_logfmt_int \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt \
  omnimorph_logfmt omnimorph_logfmt omnimorph_logfmt
};

#endif /* H_omnimorph_CODELS */
