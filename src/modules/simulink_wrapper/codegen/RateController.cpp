/*
 * RateController.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "RateController".
 *
 * Model version              : 1.173
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Wed Nov 14 21:26:16 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "RateController.h"
#include "RateController_private.h"

/* Model step function */
void RateControllerModelClass::step()
{
  /* local block i/o variables */
  real_T rtb_Diff[3];
  real_T rtb_wRotorsensor[4];
  real_T rtb_du_f[4];
  real_T rtb_DiscreteStateSpace1[4];
  real_T rtb_w[4];
  real_T rtb_w_e[4];
  real_T rtb_CastToDouble2[3];
  real_T rtb_CastToDouble4;
  real_T rtb_w_cmd[4];
  real_T rtb_du[4];
  real_T rtb_DiscreteStateSpace[3];
  real_T rtb_Omega_dot_f_Vz[4];
  real_T G_shrink[16];
  real_T G2[16];
  int8_T i_up_data[4];
  int32_T j;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T k;
  int32_T b_ix;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int8_T ii_data[4];
  int32_T b_idx;
  static const real_T g[16] = { 0.0, 0.0, -0.0612093, 0.0, 0.0, 0.0,
    0.065367000000000008, 0.0, 0.0, 0.0, -0.0657419, 0.0, 0.0, 0.0, 0.0654516,
    0.0 };

  static const real_T b[16] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 1.0E-6 };

  static const real_T a[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.01, 0.0, 0.0, 0.0, 0.0, 0.1 };

  real_T rtb_TSamp_o[4];
  real_T tmp[4];
  real_T rtb_TSamp;
  real_T rtb_TSamp_idx_1;
  real_T rtb_TSamp_idx_0;
  boolean_T exitg1;

  /* DiscreteStateSpace: '<Root>/Discrete State-Space' */
  {
    rtb_DiscreteStateSpace[0] = (1.0)*
      RateController_DW.DiscreteStateSpace_DSTATE[0];
    rtb_DiscreteStateSpace[1] = (1.0)*
      RateController_DW.DiscreteStateSpace_DSTATE[1];
    rtb_DiscreteStateSpace[2] = (1.0)*
      RateController_DW.DiscreteStateSpace_DSTATE[2];
  }

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp = rtb_DiscreteStateSpace[0] * 500.0;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  rtb_Diff[0] = rtb_TSamp - RateController_DW.UD_DSTATE[0];

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_idx_0 = rtb_TSamp;
  rtb_TSamp = rtb_DiscreteStateSpace[1] * 500.0;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  rtb_Diff[1] = rtb_TSamp - RateController_DW.UD_DSTATE[1];

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_idx_1 = rtb_TSamp;
  rtb_TSamp = rtb_DiscreteStateSpace[2] * 500.0;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  rtb_Diff[2] = rtb_TSamp - RateController_DW.UD_DSTATE[2];

  /* DiscreteStateSpace: '<Root>/wRotor sensor' */
  {
    rtb_wRotorsensor[0] = (1.0)*RateController_DW.wRotorsensor_DSTATE[0];
    rtb_wRotorsensor[1] = (1.0)*RateController_DW.wRotorsensor_DSTATE[1];
    rtb_wRotorsensor[2] = (1.0)*RateController_DW.wRotorsensor_DSTATE[2];
    rtb_wRotorsensor[3] = (1.0)*RateController_DW.wRotorsensor_DSTATE[3];
  }

  /* SignalConversion: '<S5>/TmpSignal ConversionAt SFunction Inport4' incorporates:
   *  DataTypeConversion: '<Root>/Cast To Double'
   *  Inport: '<Root>/rates_dot_sp'
   *  Inport: '<Root>/thrust_sp'
   *  MATLAB Function: '<Root>/INDI_allocator'
   */
  rtb_Omega_dot_f_Vz[0] = RateController_U.rates_dot_sp[0];
  rtb_Omega_dot_f_Vz[1] = RateController_U.rates_dot_sp[1];
  rtb_Omega_dot_f_Vz[2] = RateController_U.rates_dot_sp[2];
  rtb_Omega_dot_f_Vz[3] = RateController_U.thrust_sp;

  /* MATLAB Function: '<Root>/INDI_allocator' incorporates:
   *  Constant: '<Root>/control_eff.init'
   *  Memory: '<Root>/Memory'
   *  Sum: '<Root>/Sum of Elements'
   */
  /* :  w_max = 1256; */
  /* :  w_min = 300; */
  /* :  thrust_cmd = nu(4); */
  /* :  omega_sum_ref = 4*(thrust_cmd*(w_max - w_min) + w_min); */
  /* :  nu(4) = omega_sum_ref; */
  rtb_TSamp_o[0] = rtb_Omega_dot_f_Vz[0];
  memcpy(&G_shrink[0], &RateController_ConstP.pooled1[0], sizeof(real_T) << 4U);
  memcpy(&G2[0], &g[0], sizeof(real_T) << 4U);
  G_shrink[2] = 0.01;
  rtb_TSamp_o[1] = rtb_Omega_dot_f_Vz[1];
  G_shrink[6] = -0.01;
  rtb_TSamp_o[2] = rtb_Omega_dot_f_Vz[2];
  G_shrink[10] = 0.01;
  G_shrink[14] = -0.01;
  rtb_TSamp_o[3] = (rtb_Omega_dot_f_Vz[3] * 956.0 + 300.0) * 4.0;

  /* :  G(3,:) = par.sihao.signr*G(3,:); */
  /* :  if fail_id~=0 */
  /* :  else */
  /* :  G2 = [0 0 0 0; */
  /* :            0 0 0 0; */
  /* :            -61.2093,65.3670,-65.7419, 65.4516; */
  /* :            0 0 0 0]; */
  /* :  G2 = G2/1000; */
  /* :  G2(3,:) = par.sihao.signr*G2(3,:); */
  G2[2] = 0.0612093;
  G2[6] = -0.065367000000000008;
  G2[10] = 0.0657419;
  G2[14] = -0.0654516;

  /* :  dnu = nu - [Omega_f_dot; omega_sum]; */
  /* :  du = double((G/1000+G2)\(dnu+G2*du_last)*2*pi/60); */
  for (j = 0; j < 16; j++) {
    G_shrink[j] = G_shrink[j] / 1000.0 + G2[j];
  }

  ii_data[0] = 1;
  ii_data[1] = 2;
  ii_data[2] = 3;
  for (j = 0; j < 3; j++) {
    b_idx = j * 5;
    b_ix = 0;
    ix = b_idx;
    smax = std::abs(G_shrink[b_idx]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      s = std::abs(G_shrink[ix]);
      if (s > smax) {
        b_ix = k - 1;
        smax = s;
      }
    }

    if (G_shrink[b_idx + b_ix] != 0.0) {
      if (b_ix != 0) {
        ix = j + b_ix;
        ii_data[j] = (int8_T)(ix + 1);
        smax = G_shrink[j];
        G_shrink[j] = G_shrink[ix];
        G_shrink[ix] = smax;
        b_ix = j + 4;
        ix += 4;
        smax = G_shrink[b_ix];
        G_shrink[b_ix] = G_shrink[ix];
        G_shrink[ix] = smax;
        b_ix += 4;
        ix += 4;
        smax = G_shrink[b_ix];
        G_shrink[b_ix] = G_shrink[ix];
        G_shrink[ix] = smax;
        b_ix += 4;
        ix += 4;
        smax = G_shrink[b_ix];
        G_shrink[b_ix] = G_shrink[ix];
        G_shrink[ix] = smax;
      }

      b_ix = (b_idx - j) + 4;
      for (ix = b_idx + 1; ix < b_ix; ix++) {
        G_shrink[ix] /= G_shrink[b_idx];
      }
    }

    b_ix = b_idx + 5;
    ix = b_idx + 4;
    for (k = 0; k <= 2 - j; k++) {
      smax = G_shrink[ix];
      if (G_shrink[ix] != 0.0) {
        c_ix = b_idx + 1;
        d = (b_ix - j) + 3;
        for (ijA = b_ix; ijA < d; ijA++) {
          G_shrink[ijA] += G_shrink[c_ix] * -smax;
          c_ix++;
        }
      }

      ix += 4;
      b_ix += 4;
    }

    tmp[j] = rtb_Diff[j];
  }

  tmp[3] = ((rtb_wRotorsensor[0] + rtb_wRotorsensor[1]) + rtb_wRotorsensor[2]) +
    rtb_wRotorsensor[3];
  for (j = 0; j < 4; j++) {
    rtb_du[j] = (((G2[j + 4] * RateController_DW.Memory_PreviousInput[1] + G2[j]
                   * RateController_DW.Memory_PreviousInput[0]) + G2[j + 8] *
                  RateController_DW.Memory_PreviousInput[2]) + G2[j + 12] *
                 RateController_DW.Memory_PreviousInput[3]) + (rtb_TSamp_o[j] -
      tmp[j]);
  }

  if (ii_data[0] != 1) {
    smax = rtb_du[0];
    ix = ii_data[0] - 1;
    rtb_du[0] = rtb_du[ix];
    rtb_du[ix] = smax;
  }

  if (ii_data[1] != 2) {
    smax = rtb_du[1];
    ix = ii_data[1] - 1;
    rtb_du[1] = rtb_du[ix];
    rtb_du[ix] = smax;
  }

  if (ii_data[2] != 3) {
    smax = rtb_du[2];
    ix = ii_data[2] - 1;
    rtb_du[2] = rtb_du[ix];
    rtb_du[ix] = smax;
  }

  if (rtb_du[0] != 0.0) {
    for (j = 1; j + 1 < 5; j++) {
      rtb_du[j] -= rtb_du[0] * G_shrink[j];
    }
  }

  if (rtb_du[1] != 0.0) {
    for (j = 2; j + 1 < 5; j++) {
      rtb_du[j] -= G_shrink[j + 4] * rtb_du[1];
    }
  }

  if (rtb_du[2] != 0.0) {
    for (j = 3; j + 1 < 5; j++) {
      rtb_du[j] -= G_shrink[j + 8] * rtb_du[2];
    }
  }

  if (rtb_du[3] != 0.0) {
    rtb_du[3] /= G_shrink[15];
    for (j = 0; j < 3; j++) {
      rtb_du[j] -= G_shrink[j + 12] * rtb_du[3];
    }
  }

  if (rtb_du[2] != 0.0) {
    rtb_du[2] /= G_shrink[10];
    for (j = 0; j < 2; j++) {
      rtb_du[j] -= G_shrink[j + 8] * rtb_du[2];
    }
  }

  if (rtb_du[1] != 0.0) {
    rtb_du[1] /= G_shrink[5];
    rtb_du[0] -= rtb_du[1] * G_shrink[4];
  }

  if (rtb_du[0] != 0.0) {
    rtb_du[0] /= G_shrink[0];
  }

  rtb_du[0] = rtb_du[0] * 2.0 * 3.1415926535897931 / 60.0;
  rtb_du[1] = rtb_du[1] * 2.0 * 3.1415926535897931 / 60.0;
  rtb_du[2] = rtb_du[2] * 2.0 * 3.1415926535897931 / 60.0;
  rtb_du[3] = rtb_du[3] * 2.0 * 3.1415926535897931 / 60.0;

  /* :  w_cmd = w_f + du; */
  rtb_w_cmd[0] = rtb_wRotorsensor[0] + rtb_du[0];
  rtb_w_cmd[1] = rtb_wRotorsensor[1] + rtb_du[1];
  rtb_w_cmd[2] = rtb_wRotorsensor[2] + rtb_du[2];
  rtb_w_cmd[3] = rtb_wRotorsensor[3] + rtb_du[3];

  /* :  i_up = find(w_cmd>=w_max); */
  b_idx = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    if (rtb_w_cmd[j] >= 1256.0) {
      b_idx++;
      ii_data[b_idx - 1] = (int8_T)(j + 1);
      if (b_idx >= 4) {
        exitg1 = true;
      } else {
        j++;
      }
    } else {
      j++;
    }
  }

  if (1 > b_idx) {
    b_idx = 0;
  }

  b_ix = b_idx;
  if (0 <= b_idx - 1) {
    memcpy(&i_up_data[0], &ii_data[0], b_idx * sizeof(int8_T));
  }

  /* :  i_down = find(w_cmd<w_min); */
  b_idx = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    if (rtb_w_cmd[j] < 300.0) {
      b_idx++;
      ii_data[b_idx - 1] = (int8_T)(j + 1);
      if (b_idx >= 4) {
        exitg1 = true;
      } else {
        j++;
      }
    } else {
      j++;
    }
  }

  if (1 > b_idx) {
    b_idx = 0;
  }

  /* :  du(i_up) = w_max - w_f(i_up); */
  for (j = 0; j < b_ix; j++) {
    ix = i_up_data[j] - 1;
    rtb_du[ix] = 1256.0 - rtb_wRotorsensor[ix];
  }

  /* :  du(i_down) =  w_min - w_f(i_down); */
  for (j = 0; j < b_idx; j++) {
    ix = ii_data[j] - 1;
    rtb_du[ix] = 300.0 - rtb_wRotorsensor[ix];
  }

  /* :  w_cmd(i_up) = w_max; */
  for (j = 0; j < b_ix; j++) {
    rtb_w_cmd[i_up_data[j] - 1] = 1256.0;
  }

  /* :  w_cmd(i_down) = w_min; */
  for (j = 0; j < b_idx; j++) {
    rtb_w_cmd[ii_data[j] - 1] = 300.0;
  }

  /* Outport: '<Root>/w_cmd' */
  /* :  if fail_id~=0 */
  /* :  w_cmd_px4 = -1 + 2*(w_cmd - w_min)./(w_max - w_min); */
  RateController_Y.w_cmd[0] = rtb_w_cmd[0];

  /* Outport: '<Root>/actuators_control' incorporates:
   *  DataTypeConversion: '<Root>/Cast To Single'
   *  MATLAB Function: '<Root>/INDI_allocator'
   */
  RateController_Y.actuators_control[0] = (real32_T)((rtb_w_cmd[0] - 300.0) *
    2.0 / 956.0 + -1.0);

  /* Outport: '<Root>/w_cmd' */
  RateController_Y.w_cmd[1] = rtb_w_cmd[1];

  /* Outport: '<Root>/actuators_control' incorporates:
   *  DataTypeConversion: '<Root>/Cast To Single'
   *  MATLAB Function: '<Root>/INDI_allocator'
   */
  RateController_Y.actuators_control[1] = (real32_T)((rtb_w_cmd[1] - 300.0) *
    2.0 / 956.0 + -1.0);

  /* Outport: '<Root>/w_cmd' */
  RateController_Y.w_cmd[2] = rtb_w_cmd[2];

  /* Outport: '<Root>/actuators_control' incorporates:
   *  DataTypeConversion: '<Root>/Cast To Single'
   *  MATLAB Function: '<Root>/INDI_allocator'
   */
  RateController_Y.actuators_control[2] = (real32_T)((rtb_w_cmd[2] - 300.0) *
    2.0 / 956.0 + -1.0);

  /* Outport: '<Root>/w_cmd' */
  RateController_Y.w_cmd[3] = rtb_w_cmd[3];

  /* Outport: '<Root>/actuators_control' incorporates:
   *  DataTypeConversion: '<Root>/Cast To Single'
   *  MATLAB Function: '<Root>/INDI_allocator'
   */
  RateController_Y.actuators_control[3] = (real32_T)((rtb_w_cmd[3] - 300.0) *
    2.0 / 956.0 + -1.0);

  /* DiscreteStateSpace: '<Root>/Discrete State-Space2' */
  {
    rtb_du_f[0] = (1.0)*RateController_DW.DiscreteStateSpace2_DSTATE[0];
    rtb_du_f[1] = (1.0)*RateController_DW.DiscreteStateSpace2_DSTATE[1];
    rtb_du_f[2] = (1.0)*RateController_DW.DiscreteStateSpace2_DSTATE[2];
    rtb_du_f[3] = (1.0)*RateController_DW.DiscreteStateSpace2_DSTATE[3];
  }

  /* DiscreteStateSpace: '<Root>/Discrete State-Space3' */
  {
    rtb_Omega_dot_f_Vz[0] = (1.0)*RateController_DW.DiscreteStateSpace3_DSTATE[0];
    rtb_Omega_dot_f_Vz[1] = (1.0)*RateController_DW.DiscreteStateSpace3_DSTATE[1];
    rtb_Omega_dot_f_Vz[2] = (1.0)*RateController_DW.DiscreteStateSpace3_DSTATE[2];
    rtb_Omega_dot_f_Vz[3] = (1.0)*RateController_DW.DiscreteStateSpace3_DSTATE[3];
  }

  /* :  G_limit = 30; */
  /* :  mu_1 = diag([1 1 1 1])/1e6; */
  /* :  mu_2 = diag([1 1 1e-2 1e-1]); */
  /* :  G1 = G0 - mu_2*( G0*du_f - Omega_ddot_f_Vz )*du_f'*mu_1; */
  for (ix = 0; ix < 4; ix++) {
    /* SampleTimeMath: '<S2>/TSamp'
     *
     * About '<S2>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    smax = rtb_Omega_dot_f_Vz[ix] * 500.0;

    /* MATLAB Function: '<Root>/lms' incorporates:
     *  Memory: '<Root>/Memory3'
     *  Sum: '<S2>/Diff'
     *  UnitDelay: '<S2>/UD'
     */
    tmp[ix] = (((RateController_DW.Memory3_PreviousInput[ix + 4] * rtb_du_f[1] +
                 RateController_DW.Memory3_PreviousInput[ix] * rtb_du_f[0]) +
                RateController_DW.Memory3_PreviousInput[ix + 8] * rtb_du_f[2]) +
               RateController_DW.Memory3_PreviousInput[ix + 12] * rtb_du_f[3]) -
      (smax - RateController_DW.UD_DSTATE_n[ix]);

    /* SampleTimeMath: '<S2>/TSamp'
     *
     * About '<S2>/TSamp':
     *  y = u * K where K = 1 / ( w * Ts )
     */
    rtb_TSamp_o[ix] = smax;
  }

  /* MATLAB Function: '<Root>/lms' incorporates:
   *  Memory: '<Root>/Memory3'
   */
  for (j = 0; j < 4; j++) {
    smax = a[j + 12] * tmp[3] + (a[j + 8] * tmp[2] + (a[j + 4] * tmp[1] + a[j] *
      tmp[0]));
    G_shrink[j] = smax * rtb_du_f[0];
    G_shrink[j + 4] = smax * rtb_du_f[1];
    G_shrink[j + 8] = smax * rtb_du_f[2];
    G_shrink[j + 12] = smax * rtb_du_f[3];
    for (b_idx = 0; b_idx < 4; b_idx++) {
      b_ix = b_idx << 2;
      RateController_Y.G[j + b_ix] =
        RateController_DW.Memory3_PreviousInput[b_ix + j] - (((b[b_ix + 1] *
        G_shrink[j + 4] + b[b_ix] * G_shrink[j]) + b[b_ix + 2] * G_shrink[j + 8])
        + b[b_ix + 3] * G_shrink[j + 12]);
    }
  }

  /* :  G1(G1>G_limit) = G_limit; */
  /* :  G1(G1<-G_limit) = -G_limit; */
  for (ix = 0; ix < 16; ix++) {
    if (RateController_Y.G[ix] > 30.0) {
      RateController_Y.G[ix] = 30.0;
    }

    if (RateController_Y.G[ix] < -30.0) {
      RateController_Y.G[ix] = -30.0;
    }
  }

  /* DiscreteStateSpace: '<Root>/Discrete State-Space1' */
  /* :  mu = [1 1 1e-2 1e-1]; */
  /* :  sum = mu*abs(G); */
  {
    rtb_DiscreteStateSpace1[0] = (1.0)*
      RateController_DW.DiscreteStateSpace1_DSTATE[0];
    rtb_DiscreteStateSpace1[1] = (1.0)*
      RateController_DW.DiscreteStateSpace1_DSTATE[1];
    rtb_DiscreteStateSpace1[2] = (1.0)*
      RateController_DW.DiscreteStateSpace1_DSTATE[2];
    rtb_DiscreteStateSpace1[3] = (1.0)*
      RateController_DW.DiscreteStateSpace1_DSTATE[3];
  }

  /* DiscreteStateSpace: '<Root>/actuator dynamics' */
  {
    rtb_w[0] = (1.0)*RateController_DW.actuatordynamics_DSTATE[0];
    rtb_w[1] = (1.0)*RateController_DW.actuatordynamics_DSTATE[1];
    rtb_w[2] = (1.0)*RateController_DW.actuatordynamics_DSTATE[2];
    rtb_w[3] = (1.0)*RateController_DW.actuatordynamics_DSTATE[3];
  }

  /* DiscreteStateSpace: '<Root>/actuator dynamics1' */
  {
    rtb_w_e[0] = (1.0)*RateController_DW.actuatordynamics1_DSTATE[0];
    rtb_w_e[1] = (1.0)*RateController_DW.actuatordynamics1_DSTATE[1];
    rtb_w_e[2] = (1.0)*RateController_DW.actuatordynamics1_DSTATE[2];
    rtb_w_e[3] = (1.0)*RateController_DW.actuatordynamics1_DSTATE[3];
  }

  /* DataTypeConversion: '<Root>/Cast To Double2' incorporates:
   *  Inport: '<Root>/rates'
   */
  rtb_CastToDouble2[0] = RateController_U.rates[0];
  rtb_CastToDouble2[1] = RateController_U.rates[1];
  rtb_CastToDouble2[2] = RateController_U.rates[2];

  /* DataTypeConversion: '<Root>/Cast To Double4' incorporates:
   *  Inport: '<Root>/accel_z'
   */
  rtb_CastToDouble4 = RateController_U.accel_z;

  /* Update for DiscreteStateSpace: '<Root>/Discrete State-Space' */
  {
    real_T xnew[3];
    xnew[0] = (0.95)*RateController_DW.DiscreteStateSpace_DSTATE[0];
    xnew[0] += (0.05)*rtb_CastToDouble2[0];
    xnew[1] = (0.95)*RateController_DW.DiscreteStateSpace_DSTATE[1];
    xnew[1] += (0.05)*rtb_CastToDouble2[1];
    xnew[2] = (0.95)*RateController_DW.DiscreteStateSpace_DSTATE[2];
    xnew[2] += (0.05)*rtb_CastToDouble2[2];
    (void) memcpy(&RateController_DW.DiscreteStateSpace_DSTATE[0], xnew,
                  sizeof(real_T)*3);
  }

  /* Update for UnitDelay: '<S1>/UD' */
  RateController_DW.UD_DSTATE[0] = rtb_TSamp_idx_0;
  RateController_DW.UD_DSTATE[1] = rtb_TSamp_idx_1;
  RateController_DW.UD_DSTATE[2] = rtb_TSamp;

  /* Update for DiscreteStateSpace: '<Root>/wRotor sensor' */
  {
    real_T xnew[4];
    xnew[0] = (0.995)*RateController_DW.wRotorsensor_DSTATE[0];
    xnew[0] += (0.005)*rtb_w[0];
    xnew[1] = (0.995)*RateController_DW.wRotorsensor_DSTATE[1];
    xnew[1] += (0.005)*rtb_w[1];
    xnew[2] = (0.995)*RateController_DW.wRotorsensor_DSTATE[2];
    xnew[2] += (0.005)*rtb_w[2];
    xnew[3] = (0.995)*RateController_DW.wRotorsensor_DSTATE[3];
    xnew[3] += (0.005)*rtb_w[3];
    (void) memcpy(&RateController_DW.wRotorsensor_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for Memory: '<Root>/Memory' */
  RateController_DW.Memory_PreviousInput[0] = rtb_du[0];
  RateController_DW.Memory_PreviousInput[1] = rtb_du[1];
  RateController_DW.Memory_PreviousInput[2] = rtb_du[2];
  RateController_DW.Memory_PreviousInput[3] = rtb_du[3];

  /* Update for Memory: '<Root>/Memory3' */
  memcpy(&RateController_DW.Memory3_PreviousInput[0], &RateController_Y.G[0],
         sizeof(real_T) << 4U);

  /* Update for DiscreteStateSpace: '<Root>/Discrete State-Space2' */
  {
    real_T xnew[4];
    xnew[0] = (0.98)*RateController_DW.DiscreteStateSpace2_DSTATE[0];
    xnew[0] += (0.02)*rtb_DiscreteStateSpace1[0];
    xnew[1] = (0.98)*RateController_DW.DiscreteStateSpace2_DSTATE[1];
    xnew[1] += (0.02)*rtb_DiscreteStateSpace1[1];
    xnew[2] = (0.98)*RateController_DW.DiscreteStateSpace2_DSTATE[2];
    xnew[2] += (0.02)*rtb_DiscreteStateSpace1[2];
    xnew[3] = (0.98)*RateController_DW.DiscreteStateSpace2_DSTATE[3];
    xnew[3] += (0.02)*rtb_DiscreteStateSpace1[3];
    (void) memcpy(&RateController_DW.DiscreteStateSpace2_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<Root>/Discrete State-Space3' */
  {
    real_T xnew[4];
    xnew[0] = (0.98)*RateController_DW.DiscreteStateSpace3_DSTATE[0];
    xnew[0] += (0.02)*rtb_Diff[0];
    xnew[1] = (0.98)*RateController_DW.DiscreteStateSpace3_DSTATE[1];
    xnew[1] += (0.02)*rtb_Diff[1];
    xnew[2] = (0.98)*RateController_DW.DiscreteStateSpace3_DSTATE[2];
    xnew[2] += (0.02)*rtb_Diff[2];
    xnew[3] = (0.98)*RateController_DW.DiscreteStateSpace3_DSTATE[3];
    xnew[3] += (0.02)*rtb_CastToDouble4;
    (void) memcpy(&RateController_DW.DiscreteStateSpace3_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for UnitDelay: '<S2>/UD' */
  RateController_DW.UD_DSTATE_n[0] = rtb_TSamp_o[0];
  RateController_DW.UD_DSTATE_n[1] = rtb_TSamp_o[1];
  RateController_DW.UD_DSTATE_n[2] = rtb_TSamp_o[2];
  RateController_DW.UD_DSTATE_n[3] = rtb_TSamp_o[3];

  /* Update for DiscreteStateSpace: '<Root>/Discrete State-Space1' */
  {
    real_T xnew[4];
    xnew[0] = (0.95)*RateController_DW.DiscreteStateSpace1_DSTATE[0];
    xnew[0] += (0.05)*rtb_w_e[0];
    xnew[1] = (0.95)*RateController_DW.DiscreteStateSpace1_DSTATE[1];
    xnew[1] += (0.05)*rtb_w_e[1];
    xnew[2] = (0.95)*RateController_DW.DiscreteStateSpace1_DSTATE[2];
    xnew[2] += (0.05)*rtb_w_e[2];
    xnew[3] = (0.95)*RateController_DW.DiscreteStateSpace1_DSTATE[3];
    xnew[3] += (0.05)*rtb_w_e[3];
    (void) memcpy(&RateController_DW.DiscreteStateSpace1_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<Root>/actuator dynamics' */
  {
    real_T xnew[4];
    xnew[0] = (0.92)*RateController_DW.actuatordynamics_DSTATE[0];
    xnew[0] += (0.08)*rtb_w_cmd[0];
    xnew[1] = (0.92)*RateController_DW.actuatordynamics_DSTATE[1];
    xnew[1] += (0.08)*rtb_w_cmd[1];
    xnew[2] = (0.92)*RateController_DW.actuatordynamics_DSTATE[2];
    xnew[2] += (0.08)*rtb_w_cmd[2];
    xnew[3] = (0.92)*RateController_DW.actuatordynamics_DSTATE[3];
    xnew[3] += (0.08)*rtb_w_cmd[3];
    (void) memcpy(&RateController_DW.actuatordynamics_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<Root>/actuator dynamics1' */
  {
    real_T xnew[4];
    xnew[0] = (0.92)*RateController_DW.actuatordynamics1_DSTATE[0];
    xnew[0] += (0.08)*rtb_du[0];
    xnew[1] = (0.92)*RateController_DW.actuatordynamics1_DSTATE[1];
    xnew[1] += (0.08)*rtb_du[1];
    xnew[2] = (0.92)*RateController_DW.actuatordynamics1_DSTATE[2];
    xnew[2] += (0.08)*rtb_du[2];
    xnew[3] = (0.92)*RateController_DW.actuatordynamics1_DSTATE[3];
    xnew[3] += (0.08)*rtb_du[3];
    (void) memcpy(&RateController_DW.actuatordynamics1_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.002, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  (&RateController_M)->Timing.clockTick0++;
  if (!(&RateController_M)->Timing.clockTick0) {
    (&RateController_M)->Timing.clockTickH0++;
  }
}

/* Model initialize function */
void RateControllerModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)(&RateController_M), 0,
                sizeof(RT_MODEL_RateController_T));

  /* states (dwork) */
  (void) memset((void *)&RateController_DW, 0,
                sizeof(DW_RateController_T));

  /* external inputs */
  (void)memset(&RateController_U, 0, sizeof(ExtU_RateController_T));

  /* external outputs */
  (void) memset((void *)&RateController_Y, 0,
                sizeof(ExtY_RateController_T));

  /* InitializeConditions for DiscreteStateSpace: '<Root>/Discrete State-Space' */
  RateController_DW.DiscreteStateSpace_DSTATE[0] = 0.0;
  RateController_DW.DiscreteStateSpace_DSTATE[1] = 0.0;
  RateController_DW.DiscreteStateSpace_DSTATE[2] = 0.0;

  /* InitializeConditions for UnitDelay: '<S1>/UD' */
  RateController_DW.UD_DSTATE[0] = 0.0;
  RateController_DW.UD_DSTATE[1] = 0.0;
  RateController_DW.UD_DSTATE[2] = 0.0;

  /* InitializeConditions for DiscreteStateSpace: '<Root>/wRotor sensor' */
  RateController_DW.wRotorsensor_DSTATE[0] = (695.0);
  RateController_DW.wRotorsensor_DSTATE[1] = (695.0);
  RateController_DW.wRotorsensor_DSTATE[2] = (695.0);
  RateController_DW.wRotorsensor_DSTATE[3] = (695.0);

  /* InitializeConditions for Memory: '<Root>/Memory' */
  RateController_DW.Memory_PreviousInput[0] = 0.0;
  RateController_DW.Memory_PreviousInput[1] = 0.0;
  RateController_DW.Memory_PreviousInput[2] = 0.0;
  RateController_DW.Memory_PreviousInput[3] = 0.0;

  /* InitializeConditions for Memory: '<Root>/Memory3' */
  memcpy(&RateController_DW.Memory3_PreviousInput[0],
         &RateController_ConstP.pooled1[0], sizeof(real_T) << 4U);

  /* InitializeConditions for DiscreteStateSpace: '<Root>/Discrete State-Space2' */
  RateController_DW.DiscreteStateSpace2_DSTATE[0] = 0.0;
  RateController_DW.DiscreteStateSpace2_DSTATE[1] = 0.0;
  RateController_DW.DiscreteStateSpace2_DSTATE[2] = 0.0;
  RateController_DW.DiscreteStateSpace2_DSTATE[3] = 0.0;

  /* InitializeConditions for DiscreteStateSpace: '<Root>/Discrete State-Space3' */
  RateController_DW.DiscreteStateSpace3_DSTATE[0] = 0.0;
  RateController_DW.DiscreteStateSpace3_DSTATE[1] = 0.0;
  RateController_DW.DiscreteStateSpace3_DSTATE[2] = 0.0;
  RateController_DW.DiscreteStateSpace3_DSTATE[3] = 0.0;

  /* InitializeConditions for UnitDelay: '<S2>/UD' */
  RateController_DW.UD_DSTATE_n[0] = 0.0;
  RateController_DW.UD_DSTATE_n[1] = 0.0;
  RateController_DW.UD_DSTATE_n[2] = 0.0;
  RateController_DW.UD_DSTATE_n[3] = 0.0;

  /* InitializeConditions for DiscreteStateSpace: '<Root>/Discrete State-Space1' */
  RateController_DW.DiscreteStateSpace1_DSTATE[0] = 0.0;
  RateController_DW.DiscreteStateSpace1_DSTATE[1] = 0.0;
  RateController_DW.DiscreteStateSpace1_DSTATE[2] = 0.0;
  RateController_DW.DiscreteStateSpace1_DSTATE[3] = 0.0;

  /* InitializeConditions for DiscreteStateSpace: '<Root>/actuator dynamics' */
  RateController_DW.actuatordynamics_DSTATE[0] = 0.0;
  RateController_DW.actuatordynamics_DSTATE[1] = 0.0;
  RateController_DW.actuatordynamics_DSTATE[2] = 0.0;
  RateController_DW.actuatordynamics_DSTATE[3] = 0.0;

  /* InitializeConditions for DiscreteStateSpace: '<Root>/actuator dynamics1' */
  RateController_DW.actuatordynamics1_DSTATE[0] = 0.0;
  RateController_DW.actuatordynamics1_DSTATE[1] = 0.0;
  RateController_DW.actuatordynamics1_DSTATE[2] = 0.0;
  RateController_DW.actuatordynamics1_DSTATE[3] = 0.0;
}

/* Model terminate function */
void RateControllerModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
RateControllerModelClass::RateControllerModelClass()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
RateControllerModelClass::~RateControllerModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_RateController_T * RateControllerModelClass::getRTM()
{
  return (&RateController_M);
}
