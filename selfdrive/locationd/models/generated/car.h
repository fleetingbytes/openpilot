#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8573760947831294428);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1955944489140091225);
void car_H_mod_fun(double *state, double *out_1739137345914150167);
void car_f_fun(double *state, double dt, double *out_8515568082300461699);
void car_F_fun(double *state, double dt, double *out_6837528488726985311);
void car_h_25(double *state, double *unused, double *out_9142865966927617715);
void car_H_25(double *state, double *unused, double *out_8244420773170869748);
void car_h_24(double *state, double *unused, double *out_5856008687296022372);
void car_H_24(double *state, double *unused, double *out_4001290620687520442);
void car_h_30(double *state, double *unused, double *out_664000630063392017);
void car_H_30(double *state, double *unused, double *out_1327730431679252993);
void car_h_26(double *state, double *unused, double *out_4337672637597453628);
void car_H_26(double *state, double *unused, double *out_6460819981664625644);
void car_h_27(double *state, double *unused, double *out_6491326003512080518);
void car_H_27(double *state, double *unused, double *out_3502493743479677904);
void car_h_29(double *state, double *unused, double *out_279509222838270418);
void car_H_29(double *state, double *unused, double *out_817499087364860809);
void car_h_28(double *state, double *unused, double *out_6953744802536625453);
void car_H_28(double *state, double *unused, double *out_8148488586290792105);
void car_h_31(double *state, double *unused, double *out_2372030740577266779);
void car_H_31(double *state, double *unused, double *out_8213774811293909320);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}