#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8573760947831294428) {
   out_8573760947831294428[0] = delta_x[0] + nom_x[0];
   out_8573760947831294428[1] = delta_x[1] + nom_x[1];
   out_8573760947831294428[2] = delta_x[2] + nom_x[2];
   out_8573760947831294428[3] = delta_x[3] + nom_x[3];
   out_8573760947831294428[4] = delta_x[4] + nom_x[4];
   out_8573760947831294428[5] = delta_x[5] + nom_x[5];
   out_8573760947831294428[6] = delta_x[6] + nom_x[6];
   out_8573760947831294428[7] = delta_x[7] + nom_x[7];
   out_8573760947831294428[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1955944489140091225) {
   out_1955944489140091225[0] = -nom_x[0] + true_x[0];
   out_1955944489140091225[1] = -nom_x[1] + true_x[1];
   out_1955944489140091225[2] = -nom_x[2] + true_x[2];
   out_1955944489140091225[3] = -nom_x[3] + true_x[3];
   out_1955944489140091225[4] = -nom_x[4] + true_x[4];
   out_1955944489140091225[5] = -nom_x[5] + true_x[5];
   out_1955944489140091225[6] = -nom_x[6] + true_x[6];
   out_1955944489140091225[7] = -nom_x[7] + true_x[7];
   out_1955944489140091225[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1739137345914150167) {
   out_1739137345914150167[0] = 1.0;
   out_1739137345914150167[1] = 0;
   out_1739137345914150167[2] = 0;
   out_1739137345914150167[3] = 0;
   out_1739137345914150167[4] = 0;
   out_1739137345914150167[5] = 0;
   out_1739137345914150167[6] = 0;
   out_1739137345914150167[7] = 0;
   out_1739137345914150167[8] = 0;
   out_1739137345914150167[9] = 0;
   out_1739137345914150167[10] = 1.0;
   out_1739137345914150167[11] = 0;
   out_1739137345914150167[12] = 0;
   out_1739137345914150167[13] = 0;
   out_1739137345914150167[14] = 0;
   out_1739137345914150167[15] = 0;
   out_1739137345914150167[16] = 0;
   out_1739137345914150167[17] = 0;
   out_1739137345914150167[18] = 0;
   out_1739137345914150167[19] = 0;
   out_1739137345914150167[20] = 1.0;
   out_1739137345914150167[21] = 0;
   out_1739137345914150167[22] = 0;
   out_1739137345914150167[23] = 0;
   out_1739137345914150167[24] = 0;
   out_1739137345914150167[25] = 0;
   out_1739137345914150167[26] = 0;
   out_1739137345914150167[27] = 0;
   out_1739137345914150167[28] = 0;
   out_1739137345914150167[29] = 0;
   out_1739137345914150167[30] = 1.0;
   out_1739137345914150167[31] = 0;
   out_1739137345914150167[32] = 0;
   out_1739137345914150167[33] = 0;
   out_1739137345914150167[34] = 0;
   out_1739137345914150167[35] = 0;
   out_1739137345914150167[36] = 0;
   out_1739137345914150167[37] = 0;
   out_1739137345914150167[38] = 0;
   out_1739137345914150167[39] = 0;
   out_1739137345914150167[40] = 1.0;
   out_1739137345914150167[41] = 0;
   out_1739137345914150167[42] = 0;
   out_1739137345914150167[43] = 0;
   out_1739137345914150167[44] = 0;
   out_1739137345914150167[45] = 0;
   out_1739137345914150167[46] = 0;
   out_1739137345914150167[47] = 0;
   out_1739137345914150167[48] = 0;
   out_1739137345914150167[49] = 0;
   out_1739137345914150167[50] = 1.0;
   out_1739137345914150167[51] = 0;
   out_1739137345914150167[52] = 0;
   out_1739137345914150167[53] = 0;
   out_1739137345914150167[54] = 0;
   out_1739137345914150167[55] = 0;
   out_1739137345914150167[56] = 0;
   out_1739137345914150167[57] = 0;
   out_1739137345914150167[58] = 0;
   out_1739137345914150167[59] = 0;
   out_1739137345914150167[60] = 1.0;
   out_1739137345914150167[61] = 0;
   out_1739137345914150167[62] = 0;
   out_1739137345914150167[63] = 0;
   out_1739137345914150167[64] = 0;
   out_1739137345914150167[65] = 0;
   out_1739137345914150167[66] = 0;
   out_1739137345914150167[67] = 0;
   out_1739137345914150167[68] = 0;
   out_1739137345914150167[69] = 0;
   out_1739137345914150167[70] = 1.0;
   out_1739137345914150167[71] = 0;
   out_1739137345914150167[72] = 0;
   out_1739137345914150167[73] = 0;
   out_1739137345914150167[74] = 0;
   out_1739137345914150167[75] = 0;
   out_1739137345914150167[76] = 0;
   out_1739137345914150167[77] = 0;
   out_1739137345914150167[78] = 0;
   out_1739137345914150167[79] = 0;
   out_1739137345914150167[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8515568082300461699) {
   out_8515568082300461699[0] = state[0];
   out_8515568082300461699[1] = state[1];
   out_8515568082300461699[2] = state[2];
   out_8515568082300461699[3] = state[3];
   out_8515568082300461699[4] = state[4];
   out_8515568082300461699[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8515568082300461699[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8515568082300461699[7] = state[7];
   out_8515568082300461699[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6837528488726985311) {
   out_6837528488726985311[0] = 1;
   out_6837528488726985311[1] = 0;
   out_6837528488726985311[2] = 0;
   out_6837528488726985311[3] = 0;
   out_6837528488726985311[4] = 0;
   out_6837528488726985311[5] = 0;
   out_6837528488726985311[6] = 0;
   out_6837528488726985311[7] = 0;
   out_6837528488726985311[8] = 0;
   out_6837528488726985311[9] = 0;
   out_6837528488726985311[10] = 1;
   out_6837528488726985311[11] = 0;
   out_6837528488726985311[12] = 0;
   out_6837528488726985311[13] = 0;
   out_6837528488726985311[14] = 0;
   out_6837528488726985311[15] = 0;
   out_6837528488726985311[16] = 0;
   out_6837528488726985311[17] = 0;
   out_6837528488726985311[18] = 0;
   out_6837528488726985311[19] = 0;
   out_6837528488726985311[20] = 1;
   out_6837528488726985311[21] = 0;
   out_6837528488726985311[22] = 0;
   out_6837528488726985311[23] = 0;
   out_6837528488726985311[24] = 0;
   out_6837528488726985311[25] = 0;
   out_6837528488726985311[26] = 0;
   out_6837528488726985311[27] = 0;
   out_6837528488726985311[28] = 0;
   out_6837528488726985311[29] = 0;
   out_6837528488726985311[30] = 1;
   out_6837528488726985311[31] = 0;
   out_6837528488726985311[32] = 0;
   out_6837528488726985311[33] = 0;
   out_6837528488726985311[34] = 0;
   out_6837528488726985311[35] = 0;
   out_6837528488726985311[36] = 0;
   out_6837528488726985311[37] = 0;
   out_6837528488726985311[38] = 0;
   out_6837528488726985311[39] = 0;
   out_6837528488726985311[40] = 1;
   out_6837528488726985311[41] = 0;
   out_6837528488726985311[42] = 0;
   out_6837528488726985311[43] = 0;
   out_6837528488726985311[44] = 0;
   out_6837528488726985311[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6837528488726985311[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6837528488726985311[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6837528488726985311[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6837528488726985311[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6837528488726985311[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6837528488726985311[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6837528488726985311[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6837528488726985311[53] = -9.8000000000000007*dt;
   out_6837528488726985311[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6837528488726985311[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6837528488726985311[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6837528488726985311[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6837528488726985311[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6837528488726985311[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6837528488726985311[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6837528488726985311[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6837528488726985311[62] = 0;
   out_6837528488726985311[63] = 0;
   out_6837528488726985311[64] = 0;
   out_6837528488726985311[65] = 0;
   out_6837528488726985311[66] = 0;
   out_6837528488726985311[67] = 0;
   out_6837528488726985311[68] = 0;
   out_6837528488726985311[69] = 0;
   out_6837528488726985311[70] = 1;
   out_6837528488726985311[71] = 0;
   out_6837528488726985311[72] = 0;
   out_6837528488726985311[73] = 0;
   out_6837528488726985311[74] = 0;
   out_6837528488726985311[75] = 0;
   out_6837528488726985311[76] = 0;
   out_6837528488726985311[77] = 0;
   out_6837528488726985311[78] = 0;
   out_6837528488726985311[79] = 0;
   out_6837528488726985311[80] = 1;
}
void h_25(double *state, double *unused, double *out_9142865966927617715) {
   out_9142865966927617715[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8244420773170869748) {
   out_8244420773170869748[0] = 0;
   out_8244420773170869748[1] = 0;
   out_8244420773170869748[2] = 0;
   out_8244420773170869748[3] = 0;
   out_8244420773170869748[4] = 0;
   out_8244420773170869748[5] = 0;
   out_8244420773170869748[6] = 1;
   out_8244420773170869748[7] = 0;
   out_8244420773170869748[8] = 0;
}
void h_24(double *state, double *unused, double *out_5856008687296022372) {
   out_5856008687296022372[0] = state[4];
   out_5856008687296022372[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4001290620687520442) {
   out_4001290620687520442[0] = 0;
   out_4001290620687520442[1] = 0;
   out_4001290620687520442[2] = 0;
   out_4001290620687520442[3] = 0;
   out_4001290620687520442[4] = 1;
   out_4001290620687520442[5] = 0;
   out_4001290620687520442[6] = 0;
   out_4001290620687520442[7] = 0;
   out_4001290620687520442[8] = 0;
   out_4001290620687520442[9] = 0;
   out_4001290620687520442[10] = 0;
   out_4001290620687520442[11] = 0;
   out_4001290620687520442[12] = 0;
   out_4001290620687520442[13] = 0;
   out_4001290620687520442[14] = 1;
   out_4001290620687520442[15] = 0;
   out_4001290620687520442[16] = 0;
   out_4001290620687520442[17] = 0;
}
void h_30(double *state, double *unused, double *out_664000630063392017) {
   out_664000630063392017[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1327730431679252993) {
   out_1327730431679252993[0] = 0;
   out_1327730431679252993[1] = 0;
   out_1327730431679252993[2] = 0;
   out_1327730431679252993[3] = 0;
   out_1327730431679252993[4] = 1;
   out_1327730431679252993[5] = 0;
   out_1327730431679252993[6] = 0;
   out_1327730431679252993[7] = 0;
   out_1327730431679252993[8] = 0;
}
void h_26(double *state, double *unused, double *out_4337672637597453628) {
   out_4337672637597453628[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6460819981664625644) {
   out_6460819981664625644[0] = 0;
   out_6460819981664625644[1] = 0;
   out_6460819981664625644[2] = 0;
   out_6460819981664625644[3] = 0;
   out_6460819981664625644[4] = 0;
   out_6460819981664625644[5] = 0;
   out_6460819981664625644[6] = 0;
   out_6460819981664625644[7] = 1;
   out_6460819981664625644[8] = 0;
}
void h_27(double *state, double *unused, double *out_6491326003512080518) {
   out_6491326003512080518[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3502493743479677904) {
   out_3502493743479677904[0] = 0;
   out_3502493743479677904[1] = 0;
   out_3502493743479677904[2] = 0;
   out_3502493743479677904[3] = 1;
   out_3502493743479677904[4] = 0;
   out_3502493743479677904[5] = 0;
   out_3502493743479677904[6] = 0;
   out_3502493743479677904[7] = 0;
   out_3502493743479677904[8] = 0;
}
void h_29(double *state, double *unused, double *out_279509222838270418) {
   out_279509222838270418[0] = state[1];
}
void H_29(double *state, double *unused, double *out_817499087364860809) {
   out_817499087364860809[0] = 0;
   out_817499087364860809[1] = 1;
   out_817499087364860809[2] = 0;
   out_817499087364860809[3] = 0;
   out_817499087364860809[4] = 0;
   out_817499087364860809[5] = 0;
   out_817499087364860809[6] = 0;
   out_817499087364860809[7] = 0;
   out_817499087364860809[8] = 0;
}
void h_28(double *state, double *unused, double *out_6953744802536625453) {
   out_6953744802536625453[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8148488586290792105) {
   out_8148488586290792105[0] = 1;
   out_8148488586290792105[1] = 0;
   out_8148488586290792105[2] = 0;
   out_8148488586290792105[3] = 0;
   out_8148488586290792105[4] = 0;
   out_8148488586290792105[5] = 0;
   out_8148488586290792105[6] = 0;
   out_8148488586290792105[7] = 0;
   out_8148488586290792105[8] = 0;
}
void h_31(double *state, double *unused, double *out_2372030740577266779) {
   out_2372030740577266779[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8213774811293909320) {
   out_8213774811293909320[0] = 0;
   out_8213774811293909320[1] = 0;
   out_8213774811293909320[2] = 0;
   out_8213774811293909320[3] = 0;
   out_8213774811293909320[4] = 0;
   out_8213774811293909320[5] = 0;
   out_8213774811293909320[6] = 0;
   out_8213774811293909320[7] = 0;
   out_8213774811293909320[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_8573760947831294428) {
  err_fun(nom_x, delta_x, out_8573760947831294428);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1955944489140091225) {
  inv_err_fun(nom_x, true_x, out_1955944489140091225);
}
void car_H_mod_fun(double *state, double *out_1739137345914150167) {
  H_mod_fun(state, out_1739137345914150167);
}
void car_f_fun(double *state, double dt, double *out_8515568082300461699) {
  f_fun(state,  dt, out_8515568082300461699);
}
void car_F_fun(double *state, double dt, double *out_6837528488726985311) {
  F_fun(state,  dt, out_6837528488726985311);
}
void car_h_25(double *state, double *unused, double *out_9142865966927617715) {
  h_25(state, unused, out_9142865966927617715);
}
void car_H_25(double *state, double *unused, double *out_8244420773170869748) {
  H_25(state, unused, out_8244420773170869748);
}
void car_h_24(double *state, double *unused, double *out_5856008687296022372) {
  h_24(state, unused, out_5856008687296022372);
}
void car_H_24(double *state, double *unused, double *out_4001290620687520442) {
  H_24(state, unused, out_4001290620687520442);
}
void car_h_30(double *state, double *unused, double *out_664000630063392017) {
  h_30(state, unused, out_664000630063392017);
}
void car_H_30(double *state, double *unused, double *out_1327730431679252993) {
  H_30(state, unused, out_1327730431679252993);
}
void car_h_26(double *state, double *unused, double *out_4337672637597453628) {
  h_26(state, unused, out_4337672637597453628);
}
void car_H_26(double *state, double *unused, double *out_6460819981664625644) {
  H_26(state, unused, out_6460819981664625644);
}
void car_h_27(double *state, double *unused, double *out_6491326003512080518) {
  h_27(state, unused, out_6491326003512080518);
}
void car_H_27(double *state, double *unused, double *out_3502493743479677904) {
  H_27(state, unused, out_3502493743479677904);
}
void car_h_29(double *state, double *unused, double *out_279509222838270418) {
  h_29(state, unused, out_279509222838270418);
}
void car_H_29(double *state, double *unused, double *out_817499087364860809) {
  H_29(state, unused, out_817499087364860809);
}
void car_h_28(double *state, double *unused, double *out_6953744802536625453) {
  h_28(state, unused, out_6953744802536625453);
}
void car_H_28(double *state, double *unused, double *out_8148488586290792105) {
  H_28(state, unused, out_8148488586290792105);
}
void car_h_31(double *state, double *unused, double *out_2372030740577266779) {
  h_31(state, unused, out_2372030740577266779);
}
void car_H_31(double *state, double *unused, double *out_8213774811293909320) {
  H_31(state, unused, out_8213774811293909320);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
