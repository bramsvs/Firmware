/*
 * INDI_allocator.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "INDI_allocator".
 *
 * Model version              : 1.7
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Tue Oct  9 01:33:35 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "INDI_allocator.h"
#include "INDI_allocator_private.h"

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
void INDI_allocatorModelClass::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si
  )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  INDI_allocator_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  INDI_allocator_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  INDI_allocator_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void INDI_allocatorModelClass::step()
{
  if (rtmIsMajorTimeStep((&INDI_allocator_M))) {
    /* set solver stop time */
    if (!((&INDI_allocator_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&INDI_allocator_M)->solverInfo,
                            (((&INDI_allocator_M)->Timing.clockTickH0 + 1) *
        (&INDI_allocator_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&INDI_allocator_M)->solverInfo,
                            (((&INDI_allocator_M)->Timing.clockTick0 + 1) *
        (&INDI_allocator_M)->Timing.stepSize0 + (&INDI_allocator_M)
        ->Timing.clockTickH0 * (&INDI_allocator_M)->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&INDI_allocator_M))) {
    (&INDI_allocator_M)->Timing.t[0] = rtsiGetT(&(&INDI_allocator_M)->solverInfo);
  }

  {
    real_T (*lastU)[3];
    real_T G[16];
    real_T G2[16];
    int8_T i_up_data[4];
    int32_T ix;
    real_T smax;
    real_T s;
    int32_T jA;
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

    real_T rtb_Derivative[3];
    real_T rtb_TmpSignalConversionAtSFunct[4];
    real_T rtb_Derivative_0[4];
    int32_T ipiv_tmp;
    boolean_T exitg1;

    /* StateSpace: '<Root>/lowpass_filter ' */
    INDI_allocator_B.lowpass_filter[0] = 0.0;
    INDI_allocator_B.lowpass_filter[1] = 0.0;
    INDI_allocator_B.lowpass_filter[2] = 0.0;
    INDI_allocator_B.lowpass_filter[0] +=
      INDI_allocator_X.lowpass_filter_CSTATE[0];
    INDI_allocator_B.lowpass_filter[1] +=
      INDI_allocator_X.lowpass_filter_CSTATE[1];
    INDI_allocator_B.lowpass_filter[2] +=
      INDI_allocator_X.lowpass_filter_CSTATE[2];

    /* Derivative: '<Root>/Derivative' */
    s = (&INDI_allocator_M)->Timing.t[0];
    if ((INDI_allocator_DW.TimeStampA >= s) && (INDI_allocator_DW.TimeStampB >=
         s)) {
      rtb_Derivative[0] = 0.0;
      rtb_Derivative[1] = 0.0;
      rtb_Derivative[2] = 0.0;
    } else {
      smax = INDI_allocator_DW.TimeStampA;
      lastU = &INDI_allocator_DW.LastUAtTimeA;
      if (INDI_allocator_DW.TimeStampA < INDI_allocator_DW.TimeStampB) {
        if (INDI_allocator_DW.TimeStampB < s) {
          smax = INDI_allocator_DW.TimeStampB;
          lastU = &INDI_allocator_DW.LastUAtTimeB;
        }
      } else {
        if (INDI_allocator_DW.TimeStampA >= s) {
          smax = INDI_allocator_DW.TimeStampB;
          lastU = &INDI_allocator_DW.LastUAtTimeB;
        }
      }

      smax = s - smax;
      rtb_Derivative[0] = (INDI_allocator_B.lowpass_filter[0] - (*lastU)[0]) /
        smax;
      rtb_Derivative[1] = (INDI_allocator_B.lowpass_filter[1] - (*lastU)[1]) /
        smax;
      rtb_Derivative[2] = (INDI_allocator_B.lowpass_filter[2] - (*lastU)[2]) /
        smax;
    }

    /* End of Derivative: '<Root>/Derivative' */
    if (rtmIsMajorTimeStep((&INDI_allocator_M))) {
      /* Memory: '<Root>/Memory' */
      INDI_allocator_B.Memory[0] = INDI_allocator_DW.Memory_PreviousInput[0];
      INDI_allocator_B.Memory[1] = INDI_allocator_DW.Memory_PreviousInput[1];
      INDI_allocator_B.Memory[2] = INDI_allocator_DW.Memory_PreviousInput[2];
      INDI_allocator_B.Memory[3] = INDI_allocator_DW.Memory_PreviousInput[3];
    }

    /* SignalConversion: '<S1>/TmpSignal ConversionAt SFunction Inport3' incorporates:
     *  Inport: '<Root>/omega_sum_ref'
     *  Inport: '<Root>/pqr_dot_cmd'
     *  MATLAB Function: '<Root>/MATLAB Function'
     */
    rtb_TmpSignalConversionAtSFunct[0] = INDI_allocator_U.u_pqr[0];
    rtb_TmpSignalConversionAtSFunct[1] = INDI_allocator_U.u_pqr[1];
    rtb_TmpSignalConversionAtSFunct[2] = INDI_allocator_U.u_pqr[2];
    rtb_TmpSignalConversionAtSFunct[3] = INDI_allocator_U.omega_sum_ref;

    /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
     *  Inport: '<Root>/wRotor'
     *  Sum: '<Root>/Sum of Elements'
     */
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
      jA = 0;
      ix = kAcol;
      smax = std::abs(G[kAcol]);
      for (ipiv_tmp = 2; ipiv_tmp <= 4 - idx; ipiv_tmp++) {
        ix++;
        s = std::abs(G[ix]);
        if (s > smax) {
          jA = ipiv_tmp - 1;
          smax = s;
        }
      }

      if (G[kAcol + jA] != 0.0) {
        if (jA != 0) {
          ipiv_tmp = idx + jA;
          ii_data[idx] = (int8_T)(ipiv_tmp + 1);
          smax = G[idx];
          G[idx] = G[ipiv_tmp];
          G[ipiv_tmp] = smax;
          ix = idx + 4;
          jA = ipiv_tmp + 4;
          smax = G[ix];
          G[ix] = G[jA];
          G[jA] = smax;
          ix += 4;
          jA += 4;
          smax = G[ix];
          G[ix] = G[jA];
          G[jA] = smax;
          ix += 4;
          jA += 4;
          smax = G[ix];
          G[ix] = G[jA];
          G[jA] = smax;
        }

        jA = (kAcol - idx) + 4;
        for (ix = kAcol + 1; ix < jA; ix++) {
          G[ix] /= G[kAcol];
        }
      }

      jA = kAcol + 5;
      ix = kAcol + 4;
      for (ipiv_tmp = 0; ipiv_tmp <= 2 - idx; ipiv_tmp++) {
        smax = G[ix];
        if (G[ix] != 0.0) {
          c_ix = kAcol + 1;
          d = (jA - idx) + 3;
          for (ijA = jA; ijA < d; ijA++) {
            G[ijA] += G[c_ix] * -smax;
            c_ix++;
          }
        }

        ix += 4;
        jA += 4;
      }

      rtb_Derivative_0[idx] = rtb_Derivative[idx];
    }

    rtb_Derivative_0[3] = ((INDI_allocator_U.wRotor[0] +
      INDI_allocator_U.wRotor[1]) + INDI_allocator_U.wRotor[2]) +
      INDI_allocator_U.wRotor[3];
    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      INDI_allocator_B.du[ipiv_tmp] = (((G2[ipiv_tmp + 4] *
        INDI_allocator_B.Memory[1] + G2[ipiv_tmp] * INDI_allocator_B.Memory[0])
        + G2[ipiv_tmp + 8] * INDI_allocator_B.Memory[2]) + G2[ipiv_tmp + 12] *
        INDI_allocator_B.Memory[3]) + (rtb_TmpSignalConversionAtSFunct[ipiv_tmp]
        - rtb_Derivative_0[ipiv_tmp]);
    }

    if (ii_data[0] != 1) {
      smax = INDI_allocator_B.du[0];
      kAcol = ii_data[0] - 1;
      INDI_allocator_B.du[0] = INDI_allocator_B.du[kAcol];
      INDI_allocator_B.du[kAcol] = smax;
    }

    if (ii_data[1] != 2) {
      smax = INDI_allocator_B.du[1];
      kAcol = ii_data[1] - 1;
      INDI_allocator_B.du[1] = INDI_allocator_B.du[kAcol];
      INDI_allocator_B.du[kAcol] = smax;
    }

    if (ii_data[2] != 3) {
      smax = INDI_allocator_B.du[2];
      kAcol = ii_data[2] - 1;
      INDI_allocator_B.du[2] = INDI_allocator_B.du[kAcol];
      INDI_allocator_B.du[kAcol] = smax;
    }

    if (INDI_allocator_B.du[0] != 0.0) {
      for (jA = 1; jA + 1 < 5; jA++) {
        INDI_allocator_B.du[jA] -= INDI_allocator_B.du[0] * G[jA];
      }
    }

    if (INDI_allocator_B.du[1] != 0.0) {
      for (jA = 2; jA + 1 < 5; jA++) {
        INDI_allocator_B.du[jA] -= G[jA + 4] * INDI_allocator_B.du[1];
      }
    }

    if (INDI_allocator_B.du[2] != 0.0) {
      for (jA = 3; jA + 1 < 5; jA++) {
        INDI_allocator_B.du[jA] -= G[jA + 8] * INDI_allocator_B.du[2];
      }
    }

    if (INDI_allocator_B.du[3] != 0.0) {
      INDI_allocator_B.du[3] /= G[15];
      for (jA = 0; jA < 3; jA++) {
        INDI_allocator_B.du[jA] -= G[jA + 12] * INDI_allocator_B.du[3];
      }
    }

    if (INDI_allocator_B.du[2] != 0.0) {
      INDI_allocator_B.du[2] /= G[10];
      for (jA = 0; jA < 2; jA++) {
        INDI_allocator_B.du[jA] -= G[jA + 8] * INDI_allocator_B.du[2];
      }
    }

    if (INDI_allocator_B.du[1] != 0.0) {
      INDI_allocator_B.du[1] /= G[5];
      INDI_allocator_B.du[0] -= INDI_allocator_B.du[1] * G[4];
    }

    if (INDI_allocator_B.du[0] != 0.0) {
      INDI_allocator_B.du[0] /= G[0];
    }

    INDI_allocator_B.du[0] = INDI_allocator_B.du[0] * 2.0 * 3.1415926535897931 /
      60.0;
    INDI_allocator_B.du[1] = INDI_allocator_B.du[1] * 2.0 * 3.1415926535897931 /
      60.0;
    INDI_allocator_B.du[2] = INDI_allocator_B.du[2] * 2.0 * 3.1415926535897931 /
      60.0;
    INDI_allocator_B.du[3] = INDI_allocator_B.du[3] * 2.0 * 3.1415926535897931 /
      60.0;
    rtb_TmpSignalConversionAtSFunct[0] = INDI_allocator_U.wRotor[0] +
      INDI_allocator_B.du[0];
    rtb_TmpSignalConversionAtSFunct[1] = INDI_allocator_U.wRotor[1] +
      INDI_allocator_B.du[1];
    rtb_TmpSignalConversionAtSFunct[2] = INDI_allocator_U.wRotor[2] +
      INDI_allocator_B.du[2];
    rtb_TmpSignalConversionAtSFunct[3] = INDI_allocator_U.wRotor[3] +
      INDI_allocator_B.du[3];
    idx = 0;
    kAcol = 0;
    exitg1 = false;
    while ((!exitg1) && (kAcol < 4)) {
      if (rtb_TmpSignalConversionAtSFunct[kAcol] >= 1256.0) {
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
      if (rtb_TmpSignalConversionAtSFunct[kAcol] < 300.0) {
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
      INDI_allocator_B.du[kAcol] = 1256.0 - INDI_allocator_U.wRotor[kAcol];
    }

    for (ipiv_tmp = 0; ipiv_tmp < idx; ipiv_tmp++) {
      kAcol = ii_data[ipiv_tmp] - 1;
      INDI_allocator_B.du[kAcol] = 300.0 - INDI_allocator_U.wRotor[kAcol];
    }

    for (ipiv_tmp = 0; ipiv_tmp < ix; ipiv_tmp++) {
      rtb_TmpSignalConversionAtSFunct[i_up_data[ipiv_tmp] - 1] = 1256.0;
    }

    for (ipiv_tmp = 0; ipiv_tmp < idx; ipiv_tmp++) {
      rtb_TmpSignalConversionAtSFunct[ii_data[ipiv_tmp] - 1] = 300.0;
    }

    /* Outport: '<Root>/w_cmd' */
    INDI_allocator_Y.w_cmd[0] = rtb_TmpSignalConversionAtSFunct[0];
    INDI_allocator_Y.w_cmd[1] = rtb_TmpSignalConversionAtSFunct[1];
    INDI_allocator_Y.w_cmd[2] = rtb_TmpSignalConversionAtSFunct[2];
    INDI_allocator_Y.w_cmd[3] = rtb_TmpSignalConversionAtSFunct[3];
  }

  if (rtmIsMajorTimeStep((&INDI_allocator_M))) {
    real_T (*lastU)[3];

    /* Update for Derivative: '<Root>/Derivative' */
    if (INDI_allocator_DW.TimeStampA == (rtInf)) {
      INDI_allocator_DW.TimeStampA = (&INDI_allocator_M)->Timing.t[0];
      lastU = &INDI_allocator_DW.LastUAtTimeA;
    } else if (INDI_allocator_DW.TimeStampB == (rtInf)) {
      INDI_allocator_DW.TimeStampB = (&INDI_allocator_M)->Timing.t[0];
      lastU = &INDI_allocator_DW.LastUAtTimeB;
    } else if (INDI_allocator_DW.TimeStampA < INDI_allocator_DW.TimeStampB) {
      INDI_allocator_DW.TimeStampA = (&INDI_allocator_M)->Timing.t[0];
      lastU = &INDI_allocator_DW.LastUAtTimeA;
    } else {
      INDI_allocator_DW.TimeStampB = (&INDI_allocator_M)->Timing.t[0];
      lastU = &INDI_allocator_DW.LastUAtTimeB;
    }

    (*lastU)[0] = INDI_allocator_B.lowpass_filter[0];
    (*lastU)[1] = INDI_allocator_B.lowpass_filter[1];
    (*lastU)[2] = INDI_allocator_B.lowpass_filter[2];

    /* End of Update for Derivative: '<Root>/Derivative' */
    if (rtmIsMajorTimeStep((&INDI_allocator_M))) {
      /* Update for Memory: '<Root>/Memory' */
      INDI_allocator_DW.Memory_PreviousInput[0] = INDI_allocator_B.du[0];
      INDI_allocator_DW.Memory_PreviousInput[1] = INDI_allocator_B.du[1];
      INDI_allocator_DW.Memory_PreviousInput[2] = INDI_allocator_B.du[2];
      INDI_allocator_DW.Memory_PreviousInput[3] = INDI_allocator_B.du[3];
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&INDI_allocator_M))) {
    rt_ertODEUpdateContinuousStates(&(&INDI_allocator_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&INDI_allocator_M)->Timing.clockTick0)) {
      ++(&INDI_allocator_M)->Timing.clockTickH0;
    }

    (&INDI_allocator_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&INDI_allocator_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.002s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.002, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&INDI_allocator_M)->Timing.clockTick1++;
      if (!(&INDI_allocator_M)->Timing.clockTick1) {
        (&INDI_allocator_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void INDI_allocatorModelClass::INDI_allocator_derivatives()
{
  XDot_INDI_allocator_T *_rtXdot;
  _rtXdot = ((XDot_INDI_allocator_T *) (&INDI_allocator_M)->derivs);

  /* Derivatives for StateSpace: '<Root>/lowpass_filter ' incorporates:
   *  Inport: '<Root>/pqr'
   */
  _rtXdot->lowpass_filter_CSTATE[0] = 0.0;
  _rtXdot->lowpass_filter_CSTATE[1] = 0.0;
  _rtXdot->lowpass_filter_CSTATE[2] = 0.0;
  _rtXdot->lowpass_filter_CSTATE[0] += -25.0 *
    INDI_allocator_X.lowpass_filter_CSTATE[0];
  _rtXdot->lowpass_filter_CSTATE[1] += -25.0 *
    INDI_allocator_X.lowpass_filter_CSTATE[1];
  _rtXdot->lowpass_filter_CSTATE[2] += -25.0 *
    INDI_allocator_X.lowpass_filter_CSTATE[2];
  _rtXdot->lowpass_filter_CSTATE[0] += 25.0 * INDI_allocator_U.pqr[0];
  _rtXdot->lowpass_filter_CSTATE[1] += 25.0 * INDI_allocator_U.pqr[1];
  _rtXdot->lowpass_filter_CSTATE[2] += 25.0 * INDI_allocator_U.pqr[2];
}

/* Model initialize function */
void INDI_allocatorModelClass::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)(&INDI_allocator_M), 0,
                sizeof(RT_MODEL_INDI_allocator_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&INDI_allocator_M)->solverInfo, &(&INDI_allocator_M
                          )->Timing.simTimeStep);
    rtsiSetTPtr(&(&INDI_allocator_M)->solverInfo, &rtmGetTPtr((&INDI_allocator_M)));
    rtsiSetStepSizePtr(&(&INDI_allocator_M)->solverInfo, &(&INDI_allocator_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&INDI_allocator_M)->solverInfo, &(&INDI_allocator_M)->derivs);
    rtsiSetContStatesPtr(&(&INDI_allocator_M)->solverInfo, (real_T **)
                         &(&INDI_allocator_M)->contStates);
    rtsiSetNumContStatesPtr(&(&INDI_allocator_M)->solverInfo,
      &(&INDI_allocator_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&INDI_allocator_M)->solverInfo,
      &(&INDI_allocator_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&INDI_allocator_M)->solverInfo,
      &(&INDI_allocator_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&INDI_allocator_M)->solverInfo,
      &(&INDI_allocator_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&INDI_allocator_M)->solverInfo, (&rtmGetErrorStatus
      ((&INDI_allocator_M))));
    rtsiSetRTModelPtr(&(&INDI_allocator_M)->solverInfo, (&INDI_allocator_M));
  }

  rtsiSetSimTimeStep(&(&INDI_allocator_M)->solverInfo, MAJOR_TIME_STEP);
  (&INDI_allocator_M)->intgData.y = (&INDI_allocator_M)->odeY;
  (&INDI_allocator_M)->intgData.f[0] = (&INDI_allocator_M)->odeF[0];
  (&INDI_allocator_M)->intgData.f[1] = (&INDI_allocator_M)->odeF[1];
  (&INDI_allocator_M)->intgData.f[2] = (&INDI_allocator_M)->odeF[2];
  getRTM()->contStates = ((X_INDI_allocator_T *) &INDI_allocator_X);
  rtsiSetSolverData(&(&INDI_allocator_M)->solverInfo, (void *)
                    &(&INDI_allocator_M)->intgData);
  rtsiSetSolverName(&(&INDI_allocator_M)->solverInfo,"ode3");
  rtmSetTPtr(getRTM(), &(&INDI_allocator_M)->Timing.tArray[0]);
  (&INDI_allocator_M)->Timing.stepSize0 = 0.002;

  /* block I/O */
  (void) memset(((void *) &INDI_allocator_B), 0,
                sizeof(B_INDI_allocator_T));

  /* states (continuous) */
  {
    (void) memset((void *)&INDI_allocator_X, 0,
                  sizeof(X_INDI_allocator_T));
  }

  /* states (dwork) */
  (void) memset((void *)&INDI_allocator_DW, 0,
                sizeof(DW_INDI_allocator_T));

  /* external inputs */
  (void)memset(&INDI_allocator_U, 0, sizeof(ExtU_INDI_allocator_T));

  /* external outputs */
  (void) memset(&INDI_allocator_Y.w_cmd[0], 0,
                4U*sizeof(real_T));

  /* InitializeConditions for StateSpace: '<Root>/lowpass_filter ' */
  INDI_allocator_X.lowpass_filter_CSTATE[0] = 0.0;
  INDI_allocator_X.lowpass_filter_CSTATE[1] = 0.0;
  INDI_allocator_X.lowpass_filter_CSTATE[2] = 0.0;

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  INDI_allocator_DW.TimeStampA = (rtInf);
  INDI_allocator_DW.TimeStampB = (rtInf);

  /* InitializeConditions for Memory: '<Root>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[0] = 0.0;
  INDI_allocator_DW.Memory_PreviousInput[1] = 0.0;
  INDI_allocator_DW.Memory_PreviousInput[2] = 0.0;
  INDI_allocator_DW.Memory_PreviousInput[3] = 0.0;
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
