/*
 * INDI_allocator.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "INDI_allocator".
 *
 * Model version              : 1.49
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Thu Oct 18 10:11:03 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "INDI_allocator.h"
#include "INDI_allocator_private.h"

/* Model step function */
void INDI_allocatorModelClass::step()
{
  /* local block i/o variables */
  real_T rtb_wRotorsensor[4];
  real_T rtb_w[4];
  real_T rtb_w_cmd[4];
  real_T rtb_DiscreteStateSpace[3];
  real_T G[16];
  real_T G2[16];
  int8_T i_up_data[4];
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int32_T kAcol;
  int8_T ii_data[4];
  int32_T idx;
  static const real_T g[16] = { 15.0, 14.0, -0.25, 1.0, -15.0, 14.0, 0.25, 1.0,
    -15.0, -14.0, -0.25, 1.0, 15.0, -14.0, 0.25, 1.0 };

  static const real_T h[16] = { 0.0, 0.0, -0.0612093, 0.0, 0.0, 0.0,
    0.065367000000000008, 0.0, 0.0, 0.0, -0.0657419, 0.0, 0.0, 0.0, 0.0654516,
    0.0 };

  real_T rtb_TmpSignalConversionAtSFunct[4];
  real_T rtb_du[4];
  real_T rtb_Diff[3];
  real_T rtb_Diff_0[4];
  real_T rtb_TSamp;
  real_T rtb_TSamp_idx_1;
  real_T rtb_TSamp_idx_0;
  int32_T ipiv_tmp;
  boolean_T exitg1;

  /* DiscreteStateSpace: '<Root>/wRotor sensor' */
  {
    rtb_wRotorsensor[0] = (1.0)*INDI_allocator_DW.wRotorsensor_DSTATE[0];
    rtb_wRotorsensor[1] = (1.0)*INDI_allocator_DW.wRotorsensor_DSTATE[1];
    rtb_wRotorsensor[2] = (1.0)*INDI_allocator_DW.wRotorsensor_DSTATE[2];
    rtb_wRotorsensor[3] = (1.0)*INDI_allocator_DW.wRotorsensor_DSTATE[3];
  }

  /* DiscreteStateSpace: '<Root>/Discrete State-Space' incorporates:
   *  Inport: '<Root>/pqr'
   */
  {
    rtb_DiscreteStateSpace[0] = (1.0)*
      INDI_allocator_DW.DiscreteStateSpace_DSTATE[0];
    rtb_DiscreteStateSpace[1] = (1.0)*
      INDI_allocator_DW.DiscreteStateSpace_DSTATE[1];
    rtb_DiscreteStateSpace[2] = (1.0)*
      INDI_allocator_DW.DiscreteStateSpace_DSTATE[2];
  }

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp = rtb_DiscreteStateSpace[0] * 250.0;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  rtb_Diff[0] = rtb_TSamp - INDI_allocator_DW.UD_DSTATE[0];

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_idx_0 = rtb_TSamp;
  rtb_TSamp = rtb_DiscreteStateSpace[1] * 250.0;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  rtb_Diff[1] = rtb_TSamp - INDI_allocator_DW.UD_DSTATE[1];

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_idx_1 = rtb_TSamp;
  rtb_TSamp = rtb_DiscreteStateSpace[2] * 250.0;

  /* Sum: '<S1>/Diff' incorporates:
   *  UnitDelay: '<S1>/UD'
   */
  rtb_Diff[2] = rtb_TSamp - INDI_allocator_DW.UD_DSTATE[2];

  /* SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport3' incorporates:
   *  Inport: '<Root>/pqr_dot_cmd'
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  rtb_TmpSignalConversionAtSFunct[0] = INDI_allocator_U.u_pqr[0];
  rtb_TmpSignalConversionAtSFunct[1] = INDI_allocator_U.u_pqr[1];
  rtb_TmpSignalConversionAtSFunct[2] = INDI_allocator_U.u_pqr[2];

  /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
   *  Inport: '<Root>/thrust_cmd'
   *  Memory: '<Root>/Memory'
   *  Sum: '<Root>/Sum of Elements'
   */
  rtb_TmpSignalConversionAtSFunct[3] = (INDI_allocator_U.thrust_cmd * 956.0 +
    300.0) * 4.0;
  memcpy(&G[0], &g[0], sizeof(real_T) << 4U);
  memcpy(&G2[0], &h[0], sizeof(real_T) << 4U);
  G[2] = 0.25;
  G[6] = -0.25;
  G[10] = 0.25;
  G[14] = -0.25;
  G2[2] = 0.0612093;
  G2[6] = -0.065367000000000008;
  G2[10] = 0.0657419;
  G2[14] = -0.0654516;
  for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
    G[ipiv_tmp] = G[ipiv_tmp] / 1000.0 + G2[ipiv_tmp];
  }

  ii_data[0] = 1;
  ii_data[1] = 2;
  ii_data[2] = 3;
  for (idx = 0; idx < 3; idx++) {
    kAcol = idx * 5;
    iy = 0;
    ix = kAcol;
    smax = std::abs(G[kAcol]);
    for (ipiv_tmp = 2; ipiv_tmp <= 4 - idx; ipiv_tmp++) {
      ix++;
      s = std::abs(G[ix]);
      if (s > smax) {
        iy = ipiv_tmp - 1;
        smax = s;
      }
    }

    if (G[kAcol + iy] != 0.0) {
      if (iy != 0) {
        ipiv_tmp = idx + iy;
        ii_data[idx] = (int8_T)(ipiv_tmp + 1);
        smax = G[idx];
        G[idx] = G[ipiv_tmp];
        G[ipiv_tmp] = smax;
        ix = idx + 4;
        iy = ipiv_tmp + 4;
        smax = G[ix];
        G[ix] = G[iy];
        G[iy] = smax;
        ix += 4;
        iy += 4;
        smax = G[ix];
        G[ix] = G[iy];
        G[iy] = smax;
        ix += 4;
        iy += 4;
        smax = G[ix];
        G[ix] = G[iy];
        G[iy] = smax;
      }

      iy = (kAcol - idx) + 4;
      for (ix = kAcol + 1; ix < iy; ix++) {
        G[ix] /= G[kAcol];
      }
    }

    iy = kAcol + 5;
    ix = kAcol + 4;
    for (ipiv_tmp = 0; ipiv_tmp <= 2 - idx; ipiv_tmp++) {
      smax = G[ix];
      if (G[ix] != 0.0) {
        c_ix = kAcol + 1;
        d = (iy - idx) + 3;
        for (ijA = iy; ijA < d; ijA++) {
          G[ijA] += G[c_ix] * -smax;
          c_ix++;
        }
      }

      ix += 4;
      iy += 4;
    }

    rtb_Diff_0[idx] = rtb_Diff[idx];
  }

  rtb_Diff_0[3] = ((rtb_wRotorsensor[0] + rtb_wRotorsensor[1]) +
                   rtb_wRotorsensor[2]) + rtb_wRotorsensor[3];
  for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
    rtb_du[ipiv_tmp] = (((G2[ipiv_tmp + 4] *
                          INDI_allocator_DW.Memory_PreviousInput[1] +
                          G2[ipiv_tmp] * INDI_allocator_DW.Memory_PreviousInput
                          [0]) + G2[ipiv_tmp + 8] *
                         INDI_allocator_DW.Memory_PreviousInput[2]) +
                        G2[ipiv_tmp + 12] *
                        INDI_allocator_DW.Memory_PreviousInput[3]) +
      (rtb_TmpSignalConversionAtSFunct[ipiv_tmp] - rtb_Diff_0[ipiv_tmp]);
  }

  if (ii_data[0] != 1) {
    smax = rtb_du[0];
    kAcol = ii_data[0] - 1;
    rtb_du[0] = rtb_du[kAcol];
    rtb_du[kAcol] = smax;
  }

  if (ii_data[1] != 2) {
    smax = rtb_du[1];
    kAcol = ii_data[1] - 1;
    rtb_du[1] = rtb_du[kAcol];
    rtb_du[kAcol] = smax;
  }

  if (ii_data[2] != 3) {
    smax = rtb_du[2];
    kAcol = ii_data[2] - 1;
    rtb_du[2] = rtb_du[kAcol];
    rtb_du[kAcol] = smax;
  }

  if (rtb_du[0] != 0.0) {
    for (iy = 1; iy + 1 < 5; iy++) {
      rtb_du[iy] -= rtb_du[0] * G[iy];
    }
  }

  if (rtb_du[1] != 0.0) {
    for (iy = 2; iy + 1 < 5; iy++) {
      rtb_du[iy] -= G[iy + 4] * rtb_du[1];
    }
  }

  if (rtb_du[2] != 0.0) {
    for (iy = 3; iy + 1 < 5; iy++) {
      rtb_du[iy] -= G[iy + 8] * rtb_du[2];
    }
  }

  if (rtb_du[3] != 0.0) {
    rtb_du[3] /= G[15];
    for (iy = 0; iy < 3; iy++) {
      rtb_du[iy] -= G[iy + 12] * rtb_du[3];
    }
  }

  if (rtb_du[2] != 0.0) {
    rtb_du[2] /= G[10];
    for (iy = 0; iy < 2; iy++) {
      rtb_du[iy] -= G[iy + 8] * rtb_du[2];
    }
  }

  if (rtb_du[1] != 0.0) {
    rtb_du[1] /= G[5];
    rtb_du[0] -= rtb_du[1] * G[4];
  }

  kAcol = 0;
  if (rtb_du[0] != 0.0) {
    rtb_du[0] /= G[0];
  }

  rtb_du[0] = rtb_du[0] * 2.0 * 3.1415926535897931 / 60.0;
  rtb_du[1] = rtb_du[1] * 2.0 * 3.1415926535897931 / 60.0;
  rtb_du[2] = rtb_du[2] * 2.0 * 3.1415926535897931 / 60.0;
  smax = rtb_du[3] * 2.0 * 3.1415926535897931 / 60.0;
  rtb_du[3] = smax;
  rtb_w_cmd[0] = rtb_wRotorsensor[0] + rtb_du[0];
  rtb_w_cmd[1] = rtb_wRotorsensor[1] + rtb_du[1];
  rtb_w_cmd[2] = rtb_wRotorsensor[2] + rtb_du[2];
  rtb_w_cmd[3] = rtb_wRotorsensor[3] + smax;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (kAcol < 4)) {
    if (rtb_w_cmd[kAcol] >= 1256.0) {
      idx++;
      ii_data[idx - 1] = (int8_T)(kAcol + 1);
      if (idx >= 4) {
        exitg1 = true;
      } else {
        kAcol++;
      }
    } else {
      kAcol++;
    }
  }

  if (1 > idx) {
    idx = 0;
  }

  ix = idx;
  if (0 <= idx - 1) {
    memcpy(&i_up_data[0], &ii_data[0], idx * sizeof(int8_T));
  }

  idx = 0;
  kAcol = 0;
  exitg1 = false;
  while ((!exitg1) && (kAcol < 4)) {
    if (rtb_w_cmd[kAcol] < 300.0) {
      idx++;
      ii_data[idx - 1] = (int8_T)(kAcol + 1);
      if (idx >= 4) {
        exitg1 = true;
      } else {
        kAcol++;
      }
    } else {
      kAcol++;
    }
  }

  if (1 > idx) {
    idx = 0;
  }

  for (ipiv_tmp = 0; ipiv_tmp < ix; ipiv_tmp++) {
    kAcol = i_up_data[ipiv_tmp] - 1;
    rtb_du[kAcol] = 1256.0 - rtb_wRotorsensor[kAcol];
  }

  for (ipiv_tmp = 0; ipiv_tmp < idx; ipiv_tmp++) {
    kAcol = ii_data[ipiv_tmp] - 1;
    rtb_du[kAcol] = 300.0 - rtb_wRotorsensor[kAcol];
  }

  for (ipiv_tmp = 0; ipiv_tmp < ix; ipiv_tmp++) {
    rtb_w_cmd[i_up_data[ipiv_tmp] - 1] = 1256.0;
  }

  for (ipiv_tmp = 0; ipiv_tmp < idx; ipiv_tmp++) {
    rtb_w_cmd[ii_data[ipiv_tmp] - 1] = 300.0;
  }

  /* Outport: '<Root>/w_cmd' */
  INDI_allocator_Y.w_cmd[0] = rtb_w_cmd[0];

  /* Outport: '<Root>/w_cmd_px4' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  INDI_allocator_Y.w_cmd_px4[0] = (rtb_w_cmd[0] - 300.0) * 2.0 / 956.0 + -1.0;

  /* Outport: '<Root>/w_cmd' */
  INDI_allocator_Y.w_cmd[1] = rtb_w_cmd[1];

  /* Outport: '<Root>/w_cmd_px4' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  INDI_allocator_Y.w_cmd_px4[1] = (rtb_w_cmd[1] - 300.0) * 2.0 / 956.0 + -1.0;

  /* Outport: '<Root>/w_cmd' */
  INDI_allocator_Y.w_cmd[2] = rtb_w_cmd[2];

  /* Outport: '<Root>/w_cmd_px4' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  INDI_allocator_Y.w_cmd_px4[2] = (rtb_w_cmd[2] - 300.0) * 2.0 / 956.0 + -1.0;

  /* Outport: '<Root>/w_cmd' */
  INDI_allocator_Y.w_cmd[3] = rtb_w_cmd[3];

  /* Outport: '<Root>/w_cmd_px4' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function'
   */
  INDI_allocator_Y.w_cmd_px4[3] = (rtb_w_cmd[3] - 300.0) * 2.0 / 956.0 + -1.0;

  /* DiscreteStateSpace: '<Root>/actuator dynamics' */
  {
    rtb_w[0] = (1.0)*INDI_allocator_DW.actuatordynamics_DSTATE[0];
    rtb_w[1] = (1.0)*INDI_allocator_DW.actuatordynamics_DSTATE[1];
    rtb_w[2] = (1.0)*INDI_allocator_DW.actuatordynamics_DSTATE[2];
    rtb_w[3] = (1.0)*INDI_allocator_DW.actuatordynamics_DSTATE[3];
  }

  /* Update for DiscreteStateSpace: '<Root>/wRotor sensor' */
  {
    real_T xnew[4];
    xnew[0] = (0.99)*INDI_allocator_DW.wRotorsensor_DSTATE[0];
    xnew[0] += (0.01)*rtb_w[0];
    xnew[1] = (0.99)*INDI_allocator_DW.wRotorsensor_DSTATE[1];
    xnew[1] += (0.01)*rtb_w[1];
    xnew[2] = (0.99)*INDI_allocator_DW.wRotorsensor_DSTATE[2];
    xnew[2] += (0.01)*rtb_w[2];
    xnew[3] = (0.99)*INDI_allocator_DW.wRotorsensor_DSTATE[3];
    xnew[3] += (0.01)*rtb_w[3];
    (void) memcpy(&INDI_allocator_DW.wRotorsensor_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }

  /* Update for DiscreteStateSpace: '<Root>/Discrete State-Space' incorporates:
   *  Inport: '<Root>/pqr'
   */
  {
    real_T xnew[3];
    xnew[0] = (0.9)*INDI_allocator_DW.DiscreteStateSpace_DSTATE[0];
    xnew[0] += (0.1)*INDI_allocator_U.pqr[0];
    xnew[1] = (0.9)*INDI_allocator_DW.DiscreteStateSpace_DSTATE[1];
    xnew[1] += (0.1)*INDI_allocator_U.pqr[1];
    xnew[2] = (0.9)*INDI_allocator_DW.DiscreteStateSpace_DSTATE[2];
    xnew[2] += (0.1)*INDI_allocator_U.pqr[2];
    (void) memcpy(&INDI_allocator_DW.DiscreteStateSpace_DSTATE[0], xnew,
                  sizeof(real_T)*3);
  }

  /* Update for UnitDelay: '<S1>/UD' */
  INDI_allocator_DW.UD_DSTATE[0] = rtb_TSamp_idx_0;
  INDI_allocator_DW.UD_DSTATE[1] = rtb_TSamp_idx_1;
  INDI_allocator_DW.UD_DSTATE[2] = rtb_TSamp;

  /* Update for Memory: '<Root>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[0] = rtb_du[0];
  INDI_allocator_DW.Memory_PreviousInput[1] = rtb_du[1];
  INDI_allocator_DW.Memory_PreviousInput[2] = rtb_du[2];
  INDI_allocator_DW.Memory_PreviousInput[3] = rtb_du[3];

  /* Update for DiscreteStateSpace: '<Root>/actuator dynamics' */
  {
    real_T xnew[4];
    xnew[0] = (0.84)*INDI_allocator_DW.actuatordynamics_DSTATE[0];
    xnew[0] += (0.16)*rtb_w_cmd[0];
    xnew[1] = (0.84)*INDI_allocator_DW.actuatordynamics_DSTATE[1];
    xnew[1] += (0.16)*rtb_w_cmd[1];
    xnew[2] = (0.84)*INDI_allocator_DW.actuatordynamics_DSTATE[2];
    xnew[2] += (0.16)*rtb_w_cmd[2];
    xnew[3] = (0.84)*INDI_allocator_DW.actuatordynamics_DSTATE[3];
    xnew[3] += (0.16)*rtb_w_cmd[3];
    (void) memcpy(&INDI_allocator_DW.actuatordynamics_DSTATE[0], xnew,
                  sizeof(real_T)*4);
  }
}

/* Model initialize function */
void INDI_allocatorModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize error status */
  rtmSetErrorStatus(getRTM(), (NULL));

  /* states (dwork) */
  (void) memset((void *)&INDI_allocator_DW, 0,
                sizeof(DW_INDI_allocator_T));

  /* external inputs */
  (void)memset(&INDI_allocator_U, 0, sizeof(ExtU_INDI_allocator_T));

  /* external outputs */
  (void) memset((void *)&INDI_allocator_Y, 0,
                sizeof(ExtY_INDI_allocator_T));

  /* InitializeConditions for DiscreteStateSpace: '<Root>/wRotor sensor' */
  INDI_allocator_DW.wRotorsensor_DSTATE[0] = (695.0);
  INDI_allocator_DW.wRotorsensor_DSTATE[1] = (695.0);
  INDI_allocator_DW.wRotorsensor_DSTATE[2] = (695.0);
  INDI_allocator_DW.wRotorsensor_DSTATE[3] = (695.0);

  /* InitializeConditions for DiscreteStateSpace: '<Root>/Discrete State-Space' incorporates:
   *  Inport: '<Root>/pqr'
   */
  INDI_allocator_DW.DiscreteStateSpace_DSTATE[0] = 0.0;
  INDI_allocator_DW.DiscreteStateSpace_DSTATE[1] = 0.0;
  INDI_allocator_DW.DiscreteStateSpace_DSTATE[2] = 0.0;

  /* InitializeConditions for UnitDelay: '<S1>/UD' */
  INDI_allocator_DW.UD_DSTATE[0] = 0.0;
  INDI_allocator_DW.UD_DSTATE[1] = 0.0;
  INDI_allocator_DW.UD_DSTATE[2] = 0.0;

  /* InitializeConditions for Memory: '<Root>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[0] = 0.0;
  INDI_allocator_DW.Memory_PreviousInput[1] = 0.0;
  INDI_allocator_DW.Memory_PreviousInput[2] = 0.0;
  INDI_allocator_DW.Memory_PreviousInput[3] = 0.0;

  /* InitializeConditions for DiscreteStateSpace: '<Root>/actuator dynamics' */
  INDI_allocator_DW.actuatordynamics_DSTATE[0] = (695.0);
  INDI_allocator_DW.actuatordynamics_DSTATE[1] = (695.0);
  INDI_allocator_DW.actuatordynamics_DSTATE[2] = (695.0);
  INDI_allocator_DW.actuatordynamics_DSTATE[3] = (695.0);
}

/* Model terminate function */
void INDI_allocatorModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
INDI_allocatorModelClass::INDI_allocatorModelClass()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
INDI_allocatorModelClass::~INDI_allocatorModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_INDI_allocator_T * INDI_allocatorModelClass::getRTM()
{
  return (&INDI_allocator_M);
}
