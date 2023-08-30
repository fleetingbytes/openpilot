#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7606616315127893403) {
   out_7606616315127893403[0] = delta_x[0] + nom_x[0];
   out_7606616315127893403[1] = delta_x[1] + nom_x[1];
   out_7606616315127893403[2] = delta_x[2] + nom_x[2];
   out_7606616315127893403[3] = delta_x[3] + nom_x[3];
   out_7606616315127893403[4] = delta_x[4] + nom_x[4];
   out_7606616315127893403[5] = delta_x[5] + nom_x[5];
   out_7606616315127893403[6] = delta_x[6] + nom_x[6];
   out_7606616315127893403[7] = delta_x[7] + nom_x[7];
   out_7606616315127893403[8] = delta_x[8] + nom_x[8];
   out_7606616315127893403[9] = delta_x[9] + nom_x[9];
   out_7606616315127893403[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2148420408066820626) {
   out_2148420408066820626[0] = -nom_x[0] + true_x[0];
   out_2148420408066820626[1] = -nom_x[1] + true_x[1];
   out_2148420408066820626[2] = -nom_x[2] + true_x[2];
   out_2148420408066820626[3] = -nom_x[3] + true_x[3];
   out_2148420408066820626[4] = -nom_x[4] + true_x[4];
   out_2148420408066820626[5] = -nom_x[5] + true_x[5];
   out_2148420408066820626[6] = -nom_x[6] + true_x[6];
   out_2148420408066820626[7] = -nom_x[7] + true_x[7];
   out_2148420408066820626[8] = -nom_x[8] + true_x[8];
   out_2148420408066820626[9] = -nom_x[9] + true_x[9];
   out_2148420408066820626[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_7134411685623021038) {
   out_7134411685623021038[0] = 1.0;
   out_7134411685623021038[1] = 0;
   out_7134411685623021038[2] = 0;
   out_7134411685623021038[3] = 0;
   out_7134411685623021038[4] = 0;
   out_7134411685623021038[5] = 0;
   out_7134411685623021038[6] = 0;
   out_7134411685623021038[7] = 0;
   out_7134411685623021038[8] = 0;
   out_7134411685623021038[9] = 0;
   out_7134411685623021038[10] = 0;
   out_7134411685623021038[11] = 0;
   out_7134411685623021038[12] = 1.0;
   out_7134411685623021038[13] = 0;
   out_7134411685623021038[14] = 0;
   out_7134411685623021038[15] = 0;
   out_7134411685623021038[16] = 0;
   out_7134411685623021038[17] = 0;
   out_7134411685623021038[18] = 0;
   out_7134411685623021038[19] = 0;
   out_7134411685623021038[20] = 0;
   out_7134411685623021038[21] = 0;
   out_7134411685623021038[22] = 0;
   out_7134411685623021038[23] = 0;
   out_7134411685623021038[24] = 1.0;
   out_7134411685623021038[25] = 0;
   out_7134411685623021038[26] = 0;
   out_7134411685623021038[27] = 0;
   out_7134411685623021038[28] = 0;
   out_7134411685623021038[29] = 0;
   out_7134411685623021038[30] = 0;
   out_7134411685623021038[31] = 0;
   out_7134411685623021038[32] = 0;
   out_7134411685623021038[33] = 0;
   out_7134411685623021038[34] = 0;
   out_7134411685623021038[35] = 0;
   out_7134411685623021038[36] = 1.0;
   out_7134411685623021038[37] = 0;
   out_7134411685623021038[38] = 0;
   out_7134411685623021038[39] = 0;
   out_7134411685623021038[40] = 0;
   out_7134411685623021038[41] = 0;
   out_7134411685623021038[42] = 0;
   out_7134411685623021038[43] = 0;
   out_7134411685623021038[44] = 0;
   out_7134411685623021038[45] = 0;
   out_7134411685623021038[46] = 0;
   out_7134411685623021038[47] = 0;
   out_7134411685623021038[48] = 1.0;
   out_7134411685623021038[49] = 0;
   out_7134411685623021038[50] = 0;
   out_7134411685623021038[51] = 0;
   out_7134411685623021038[52] = 0;
   out_7134411685623021038[53] = 0;
   out_7134411685623021038[54] = 0;
   out_7134411685623021038[55] = 0;
   out_7134411685623021038[56] = 0;
   out_7134411685623021038[57] = 0;
   out_7134411685623021038[58] = 0;
   out_7134411685623021038[59] = 0;
   out_7134411685623021038[60] = 1.0;
   out_7134411685623021038[61] = 0;
   out_7134411685623021038[62] = 0;
   out_7134411685623021038[63] = 0;
   out_7134411685623021038[64] = 0;
   out_7134411685623021038[65] = 0;
   out_7134411685623021038[66] = 0;
   out_7134411685623021038[67] = 0;
   out_7134411685623021038[68] = 0;
   out_7134411685623021038[69] = 0;
   out_7134411685623021038[70] = 0;
   out_7134411685623021038[71] = 0;
   out_7134411685623021038[72] = 1.0;
   out_7134411685623021038[73] = 0;
   out_7134411685623021038[74] = 0;
   out_7134411685623021038[75] = 0;
   out_7134411685623021038[76] = 0;
   out_7134411685623021038[77] = 0;
   out_7134411685623021038[78] = 0;
   out_7134411685623021038[79] = 0;
   out_7134411685623021038[80] = 0;
   out_7134411685623021038[81] = 0;
   out_7134411685623021038[82] = 0;
   out_7134411685623021038[83] = 0;
   out_7134411685623021038[84] = 1.0;
   out_7134411685623021038[85] = 0;
   out_7134411685623021038[86] = 0;
   out_7134411685623021038[87] = 0;
   out_7134411685623021038[88] = 0;
   out_7134411685623021038[89] = 0;
   out_7134411685623021038[90] = 0;
   out_7134411685623021038[91] = 0;
   out_7134411685623021038[92] = 0;
   out_7134411685623021038[93] = 0;
   out_7134411685623021038[94] = 0;
   out_7134411685623021038[95] = 0;
   out_7134411685623021038[96] = 1.0;
   out_7134411685623021038[97] = 0;
   out_7134411685623021038[98] = 0;
   out_7134411685623021038[99] = 0;
   out_7134411685623021038[100] = 0;
   out_7134411685623021038[101] = 0;
   out_7134411685623021038[102] = 0;
   out_7134411685623021038[103] = 0;
   out_7134411685623021038[104] = 0;
   out_7134411685623021038[105] = 0;
   out_7134411685623021038[106] = 0;
   out_7134411685623021038[107] = 0;
   out_7134411685623021038[108] = 1.0;
   out_7134411685623021038[109] = 0;
   out_7134411685623021038[110] = 0;
   out_7134411685623021038[111] = 0;
   out_7134411685623021038[112] = 0;
   out_7134411685623021038[113] = 0;
   out_7134411685623021038[114] = 0;
   out_7134411685623021038[115] = 0;
   out_7134411685623021038[116] = 0;
   out_7134411685623021038[117] = 0;
   out_7134411685623021038[118] = 0;
   out_7134411685623021038[119] = 0;
   out_7134411685623021038[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5516062366100241664) {
   out_5516062366100241664[0] = dt*state[3] + state[0];
   out_5516062366100241664[1] = dt*state[4] + state[1];
   out_5516062366100241664[2] = dt*state[5] + state[2];
   out_5516062366100241664[3] = state[3];
   out_5516062366100241664[4] = state[4];
   out_5516062366100241664[5] = state[5];
   out_5516062366100241664[6] = dt*state[7] + state[6];
   out_5516062366100241664[7] = dt*state[8] + state[7];
   out_5516062366100241664[8] = state[8];
   out_5516062366100241664[9] = state[9];
   out_5516062366100241664[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4815759577106247656) {
   out_4815759577106247656[0] = 1;
   out_4815759577106247656[1] = 0;
   out_4815759577106247656[2] = 0;
   out_4815759577106247656[3] = dt;
   out_4815759577106247656[4] = 0;
   out_4815759577106247656[5] = 0;
   out_4815759577106247656[6] = 0;
   out_4815759577106247656[7] = 0;
   out_4815759577106247656[8] = 0;
   out_4815759577106247656[9] = 0;
   out_4815759577106247656[10] = 0;
   out_4815759577106247656[11] = 0;
   out_4815759577106247656[12] = 1;
   out_4815759577106247656[13] = 0;
   out_4815759577106247656[14] = 0;
   out_4815759577106247656[15] = dt;
   out_4815759577106247656[16] = 0;
   out_4815759577106247656[17] = 0;
   out_4815759577106247656[18] = 0;
   out_4815759577106247656[19] = 0;
   out_4815759577106247656[20] = 0;
   out_4815759577106247656[21] = 0;
   out_4815759577106247656[22] = 0;
   out_4815759577106247656[23] = 0;
   out_4815759577106247656[24] = 1;
   out_4815759577106247656[25] = 0;
   out_4815759577106247656[26] = 0;
   out_4815759577106247656[27] = dt;
   out_4815759577106247656[28] = 0;
   out_4815759577106247656[29] = 0;
   out_4815759577106247656[30] = 0;
   out_4815759577106247656[31] = 0;
   out_4815759577106247656[32] = 0;
   out_4815759577106247656[33] = 0;
   out_4815759577106247656[34] = 0;
   out_4815759577106247656[35] = 0;
   out_4815759577106247656[36] = 1;
   out_4815759577106247656[37] = 0;
   out_4815759577106247656[38] = 0;
   out_4815759577106247656[39] = 0;
   out_4815759577106247656[40] = 0;
   out_4815759577106247656[41] = 0;
   out_4815759577106247656[42] = 0;
   out_4815759577106247656[43] = 0;
   out_4815759577106247656[44] = 0;
   out_4815759577106247656[45] = 0;
   out_4815759577106247656[46] = 0;
   out_4815759577106247656[47] = 0;
   out_4815759577106247656[48] = 1;
   out_4815759577106247656[49] = 0;
   out_4815759577106247656[50] = 0;
   out_4815759577106247656[51] = 0;
   out_4815759577106247656[52] = 0;
   out_4815759577106247656[53] = 0;
   out_4815759577106247656[54] = 0;
   out_4815759577106247656[55] = 0;
   out_4815759577106247656[56] = 0;
   out_4815759577106247656[57] = 0;
   out_4815759577106247656[58] = 0;
   out_4815759577106247656[59] = 0;
   out_4815759577106247656[60] = 1;
   out_4815759577106247656[61] = 0;
   out_4815759577106247656[62] = 0;
   out_4815759577106247656[63] = 0;
   out_4815759577106247656[64] = 0;
   out_4815759577106247656[65] = 0;
   out_4815759577106247656[66] = 0;
   out_4815759577106247656[67] = 0;
   out_4815759577106247656[68] = 0;
   out_4815759577106247656[69] = 0;
   out_4815759577106247656[70] = 0;
   out_4815759577106247656[71] = 0;
   out_4815759577106247656[72] = 1;
   out_4815759577106247656[73] = dt;
   out_4815759577106247656[74] = 0;
   out_4815759577106247656[75] = 0;
   out_4815759577106247656[76] = 0;
   out_4815759577106247656[77] = 0;
   out_4815759577106247656[78] = 0;
   out_4815759577106247656[79] = 0;
   out_4815759577106247656[80] = 0;
   out_4815759577106247656[81] = 0;
   out_4815759577106247656[82] = 0;
   out_4815759577106247656[83] = 0;
   out_4815759577106247656[84] = 1;
   out_4815759577106247656[85] = dt;
   out_4815759577106247656[86] = 0;
   out_4815759577106247656[87] = 0;
   out_4815759577106247656[88] = 0;
   out_4815759577106247656[89] = 0;
   out_4815759577106247656[90] = 0;
   out_4815759577106247656[91] = 0;
   out_4815759577106247656[92] = 0;
   out_4815759577106247656[93] = 0;
   out_4815759577106247656[94] = 0;
   out_4815759577106247656[95] = 0;
   out_4815759577106247656[96] = 1;
   out_4815759577106247656[97] = 0;
   out_4815759577106247656[98] = 0;
   out_4815759577106247656[99] = 0;
   out_4815759577106247656[100] = 0;
   out_4815759577106247656[101] = 0;
   out_4815759577106247656[102] = 0;
   out_4815759577106247656[103] = 0;
   out_4815759577106247656[104] = 0;
   out_4815759577106247656[105] = 0;
   out_4815759577106247656[106] = 0;
   out_4815759577106247656[107] = 0;
   out_4815759577106247656[108] = 1;
   out_4815759577106247656[109] = 0;
   out_4815759577106247656[110] = 0;
   out_4815759577106247656[111] = 0;
   out_4815759577106247656[112] = 0;
   out_4815759577106247656[113] = 0;
   out_4815759577106247656[114] = 0;
   out_4815759577106247656[115] = 0;
   out_4815759577106247656[116] = 0;
   out_4815759577106247656[117] = 0;
   out_4815759577106247656[118] = 0;
   out_4815759577106247656[119] = 0;
   out_4815759577106247656[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3061927959196119525) {
   out_3061927959196119525[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1866541074429046865) {
   out_1866541074429046865[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1866541074429046865[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1866541074429046865[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1866541074429046865[3] = 0;
   out_1866541074429046865[4] = 0;
   out_1866541074429046865[5] = 0;
   out_1866541074429046865[6] = 1;
   out_1866541074429046865[7] = 0;
   out_1866541074429046865[8] = 0;
   out_1866541074429046865[9] = 0;
   out_1866541074429046865[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_3470832125063889419) {
   out_3470832125063889419[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_9130879236938149709) {
   out_9130879236938149709[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9130879236938149709[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9130879236938149709[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9130879236938149709[3] = 0;
   out_9130879236938149709[4] = 0;
   out_9130879236938149709[5] = 0;
   out_9130879236938149709[6] = 1;
   out_9130879236938149709[7] = 0;
   out_9130879236938149709[8] = 0;
   out_9130879236938149709[9] = 1;
   out_9130879236938149709[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_5099466840857781873) {
   out_5099466840857781873[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3246074151483163792) {
   out_3246074151483163792[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[6] = 0;
   out_3246074151483163792[7] = 1;
   out_3246074151483163792[8] = 0;
   out_3246074151483163792[9] = 0;
   out_3246074151483163792[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_5099466840857781873) {
   out_5099466840857781873[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3246074151483163792) {
   out_3246074151483163792[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3246074151483163792[6] = 0;
   out_3246074151483163792[7] = 1;
   out_3246074151483163792[8] = 0;
   out_3246074151483163792[9] = 0;
   out_3246074151483163792[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7606616315127893403) {
  err_fun(nom_x, delta_x, out_7606616315127893403);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_2148420408066820626) {
  inv_err_fun(nom_x, true_x, out_2148420408066820626);
}
void gnss_H_mod_fun(double *state, double *out_7134411685623021038) {
  H_mod_fun(state, out_7134411685623021038);
}
void gnss_f_fun(double *state, double dt, double *out_5516062366100241664) {
  f_fun(state,  dt, out_5516062366100241664);
}
void gnss_F_fun(double *state, double dt, double *out_4815759577106247656) {
  F_fun(state,  dt, out_4815759577106247656);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3061927959196119525) {
  h_6(state, sat_pos, out_3061927959196119525);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1866541074429046865) {
  H_6(state, sat_pos, out_1866541074429046865);
}
void gnss_h_20(double *state, double *sat_pos, double *out_3470832125063889419) {
  h_20(state, sat_pos, out_3470832125063889419);
}
void gnss_H_20(double *state, double *sat_pos, double *out_9130879236938149709) {
  H_20(state, sat_pos, out_9130879236938149709);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_5099466840857781873) {
  h_7(state, sat_pos_vel, out_5099466840857781873);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3246074151483163792) {
  H_7(state, sat_pos_vel, out_3246074151483163792);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_5099466840857781873) {
  h_21(state, sat_pos_vel, out_5099466840857781873);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3246074151483163792) {
  H_21(state, sat_pos_vel, out_3246074151483163792);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
