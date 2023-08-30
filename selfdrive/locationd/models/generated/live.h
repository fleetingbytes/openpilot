#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8714353098324573288);
void live_err_fun(double *nom_x, double *delta_x, double *out_3390447180956984316);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6050591331665856977);
void live_H_mod_fun(double *state, double *out_6584064886042052704);
void live_f_fun(double *state, double dt, double *out_8896929047166450191);
void live_F_fun(double *state, double dt, double *out_6591234296711432517);
void live_h_4(double *state, double *unused, double *out_17249495657416169);
void live_H_4(double *state, double *unused, double *out_2862251438192774466);
void live_h_9(double *state, double *unused, double *out_2084619982870984247);
void live_H_9(double *state, double *unused, double *out_3103441084822365111);
void live_h_10(double *state, double *unused, double *out_3447774714965570930);
void live_H_10(double *state, double *unused, double *out_73537383094869577);
void live_h_12(double *state, double *unused, double *out_6015498378302342268);
void live_H_12(double *state, double *unused, double *out_7881707846224736261);
void live_h_35(double *state, double *unused, double *out_5839578915844121242);
void live_H_35(double *state, double *unused, double *out_7819473195159801646);
void live_h_32(double *state, double *unused, double *out_2667490022277895409);
void live_H_32(double *state, double *unused, double *out_5114448682258051709);
void live_h_13(double *state, double *unused, double *out_377278173860008742);
void live_H_13(double *state, double *unused, double *out_7289401960388689286);
void live_h_14(double *state, double *unused, double *out_2084619982870984247);
void live_H_14(double *state, double *unused, double *out_3103441084822365111);
void live_h_33(double *state, double *unused, double *out_8818790185335810067);
void live_H_33(double *state, double *unused, double *out_9067273573505312170);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}