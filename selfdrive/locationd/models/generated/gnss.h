#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7606616315127893403);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2148420408066820626);
void gnss_H_mod_fun(double *state, double *out_7134411685623021038);
void gnss_f_fun(double *state, double dt, double *out_5516062366100241664);
void gnss_F_fun(double *state, double dt, double *out_4815759577106247656);
void gnss_h_6(double *state, double *sat_pos, double *out_3061927959196119525);
void gnss_H_6(double *state, double *sat_pos, double *out_1866541074429046865);
void gnss_h_20(double *state, double *sat_pos, double *out_3470832125063889419);
void gnss_H_20(double *state, double *sat_pos, double *out_9130879236938149709);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5099466840857781873);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3246074151483163792);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5099466840857781873);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3246074151483163792);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}