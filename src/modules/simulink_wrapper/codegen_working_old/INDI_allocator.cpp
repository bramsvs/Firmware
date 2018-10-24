/*
 * INDI_allocator.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "INDI_allocator".
 *
 * Model version              : 1.6830
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Tue Oct  2 03:48:13 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "INDI_allocator.h"
#include "INDI_allocator_private.h"

static void rate_scheduler(RT_MODEL_INDI_allocator_T *const INDI_allocator_M);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(RT_MODEL_INDI_allocator_T *const INDI_allocator_M)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (INDI_allocator_M->Timing.TaskCounters.TID[2])++;
  if ((INDI_allocator_M->Timing.TaskCounters.TID[2]) > 1) {/* Sample time: [0.004s, 0.0s] */
    INDI_allocator_M->Timing.TaskCounters.TID[2] = 0;
  }
}

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
  int_T nXc = 27;
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
    real_T (*lastU_0)[4];
    static const real_T b[16] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
      0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 1.0E-6 };

    static const real_T a[16] = { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.1 };

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
    int32_T b_kAcol;
    int8_T ii_data[4];
    int32_T idx;
    static const real_T g[16] = { 15.0, 14.0, -0.25, 1.0, -15.0, 14.0, 0.25, 1.0,
      -15.0, -14.0, -0.25, 1.0, 15.0, -14.0, 0.25, 1.0 };

    static const real_T h[16] = { 0.0, 0.0, -0.0612093, 0.0, 0.0, 0.0,
      0.065367000000000008, 0.0, 0.0, 0.0, -0.0657419, 0.0, 0.0, 0.0, 0.0654516,
      0.0 };

    real_T rtb_Derivative2[4];
    real_T rtb_lowpass_filter_est1[4];
    real_T rtb_TmpSignalConversionAtSFunct[4];
    real_T tmp[4];
    real_T rtb_lowpass_filter_est1_0;
    real_T rtb_lowpass_filter_est1_1;
    real_T rtb_lowpass_filter_est1_2;
    int32_T ipiv_tmp;
    boolean_T exitg1;

    /* Outputs for Atomic SubSystem: '<Root>/INDI_allocator' */
    /* StateSpace: '<S1>/lowpass_filter ' */
    for (jA = 0; jA < 3; jA++) {
      INDI_allocator_B.lowpass_filter[jA] = 0.0;
      INDI_allocator_B.lowpass_filter[jA] +=
        INDI_allocator_P.lowpass_filter_C[jA] *
        INDI_allocator_X.lowpass_filter_CSTATE[0];
      INDI_allocator_B.lowpass_filter[jA] += INDI_allocator_P.lowpass_filter_C[3
        + jA] * INDI_allocator_X.lowpass_filter_CSTATE[1];
      INDI_allocator_B.lowpass_filter[jA] += INDI_allocator_P.lowpass_filter_C[6
        + jA] * INDI_allocator_X.lowpass_filter_CSTATE[2];
    }

    /* End of StateSpace: '<S1>/lowpass_filter ' */

    /* Derivative: '<S1>/Derivative' incorporates:
     *  Derivative: '<S1>/Derivative2'
     */
    rtb_lowpass_filter_est1_0 = (&INDI_allocator_M)->Timing.t[0];
    if ((INDI_allocator_DW.TimeStampA >= rtb_lowpass_filter_est1_0) &&
        (INDI_allocator_DW.TimeStampB >= rtb_lowpass_filter_est1_0)) {
      INDI_allocator_B.Derivative[0] = 0.0;
      INDI_allocator_B.Derivative[1] = 0.0;
      INDI_allocator_B.Derivative[2] = 0.0;
    } else {
      smax = INDI_allocator_DW.TimeStampA;
      lastU = &INDI_allocator_DW.LastUAtTimeA;
      if (INDI_allocator_DW.TimeStampA < INDI_allocator_DW.TimeStampB) {
        if (INDI_allocator_DW.TimeStampB < rtb_lowpass_filter_est1_0) {
          smax = INDI_allocator_DW.TimeStampB;
          lastU = &INDI_allocator_DW.LastUAtTimeB;
        }
      } else {
        if (INDI_allocator_DW.TimeStampA >= rtb_lowpass_filter_est1_0) {
          smax = INDI_allocator_DW.TimeStampB;
          lastU = &INDI_allocator_DW.LastUAtTimeB;
        }
      }

      smax = rtb_lowpass_filter_est1_0 - smax;
      INDI_allocator_B.Derivative[0] = (INDI_allocator_B.lowpass_filter[0] -
        (*lastU)[0]) / smax;
      INDI_allocator_B.Derivative[1] = (INDI_allocator_B.lowpass_filter[1] -
        (*lastU)[1]) / smax;
      INDI_allocator_B.Derivative[2] = (INDI_allocator_B.lowpass_filter[2] -
        (*lastU)[2]) / smax;
    }

    /* End of Derivative: '<S1>/Derivative' */

    /* StateSpace: '<S1>/lowpass_filter_est ' */
    for (jA = 0; jA < 4; jA++) {
      INDI_allocator_B.lowpass_filter_est[jA] = 0.0;
      INDI_allocator_B.lowpass_filter_est[jA] +=
        INDI_allocator_P.lowpass_filter_est_C[jA] *
        INDI_allocator_X.lowpass_filter_est_CSTATE[0];
      INDI_allocator_B.lowpass_filter_est[jA] +=
        INDI_allocator_P.lowpass_filter_est_C[4 + jA] *
        INDI_allocator_X.lowpass_filter_est_CSTATE[1];
      INDI_allocator_B.lowpass_filter_est[jA] +=
        INDI_allocator_P.lowpass_filter_est_C[8 + jA] *
        INDI_allocator_X.lowpass_filter_est_CSTATE[2];
      INDI_allocator_B.lowpass_filter_est[jA] +=
        INDI_allocator_P.lowpass_filter_est_C[12 + jA] *
        INDI_allocator_X.lowpass_filter_est_CSTATE[3];
    }

    /* End of StateSpace: '<S1>/lowpass_filter_est ' */

    /* Derivative: '<S1>/Derivative2' */
    if ((INDI_allocator_DW.TimeStampA_n >= rtb_lowpass_filter_est1_0) &&
        (INDI_allocator_DW.TimeStampB_m >= rtb_lowpass_filter_est1_0)) {
      rtb_Derivative2[0] = 0.0;
      rtb_Derivative2[1] = 0.0;
      rtb_Derivative2[2] = 0.0;
      rtb_Derivative2[3] = 0.0;
    } else {
      smax = INDI_allocator_DW.TimeStampA_n;
      lastU_0 = &INDI_allocator_DW.LastUAtTimeA_c;
      if (INDI_allocator_DW.TimeStampA_n < INDI_allocator_DW.TimeStampB_m) {
        if (INDI_allocator_DW.TimeStampB_m < rtb_lowpass_filter_est1_0) {
          smax = INDI_allocator_DW.TimeStampB_m;
          lastU_0 = &INDI_allocator_DW.LastUAtTimeB_p;
        }
      } else {
        if (INDI_allocator_DW.TimeStampA_n >= rtb_lowpass_filter_est1_0) {
          smax = INDI_allocator_DW.TimeStampB_m;
          lastU_0 = &INDI_allocator_DW.LastUAtTimeB_p;
        }
      }

      smax = rtb_lowpass_filter_est1_0 - smax;
      rtb_Derivative2[0] = (INDI_allocator_B.lowpass_filter_est[0] - (*lastU_0)
                            [0]) / smax;
      rtb_Derivative2[1] = (INDI_allocator_B.lowpass_filter_est[1] - (*lastU_0)
                            [1]) / smax;
      rtb_Derivative2[2] = (INDI_allocator_B.lowpass_filter_est[2] - (*lastU_0)
                            [2]) / smax;
      rtb_Derivative2[3] = (INDI_allocator_B.lowpass_filter_est[3] - (*lastU_0)
                            [3]) / smax;
    }

    if (rtmIsMajorTimeStep((&INDI_allocator_M)) &&
        (&INDI_allocator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S1>/Memory3' */
      memcpy(&INDI_allocator_B.Memory3[0],
             &INDI_allocator_DW.Memory3_PreviousInput[0], sizeof(real_T) << 4U);
    }

    /* StateSpace: '<S1>/lowpass_filter_est 1' */
    for (jA = 0; jA < 4; jA++) {
      smax = INDI_allocator_P.lowpass_filter_est1_C[12 + jA] *
        INDI_allocator_X.lowpass_filter_est1_CSTATE[3] +
        (INDI_allocator_P.lowpass_filter_est1_C[8 + jA] *
         INDI_allocator_X.lowpass_filter_est1_CSTATE[2] +
         (INDI_allocator_P.lowpass_filter_est1_C[4 + jA] *
          INDI_allocator_X.lowpass_filter_est1_CSTATE[1] +
          INDI_allocator_P.lowpass_filter_est1_C[jA] *
          INDI_allocator_X.lowpass_filter_est1_CSTATE[0]));
      rtb_lowpass_filter_est1[jA] = smax;
    }

    /* End of StateSpace: '<S1>/lowpass_filter_est 1' */

    /* MATLAB Function: '<S1>/MATLAB Function2' */
    /* MATLAB Function 'INDI_allocator/MATLAB Function2': '<S5>:1' */
    /* '<S5>:1:16' */
    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      tmp[ipiv_tmp] = (((INDI_allocator_B.Memory3[ipiv_tmp + 4] *
                         rtb_lowpass_filter_est1[1] +
                         INDI_allocator_B.Memory3[ipiv_tmp] *
                         rtb_lowpass_filter_est1[0]) +
                        INDI_allocator_B.Memory3[ipiv_tmp + 8] *
                        rtb_lowpass_filter_est1[2]) +
                       INDI_allocator_B.Memory3[ipiv_tmp + 12] *
                       rtb_lowpass_filter_est1[3]) - rtb_Derivative2[ipiv_tmp];
    }

    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      s = a[ipiv_tmp + 12] * tmp[3] + (a[ipiv_tmp + 8] * tmp[2] + (a[ipiv_tmp +
        4] * tmp[1] + a[ipiv_tmp] * tmp[0]));
      G[ipiv_tmp] = s * rtb_lowpass_filter_est1[0];
      G[ipiv_tmp + 4] = s * rtb_lowpass_filter_est1[1];
      G[ipiv_tmp + 8] = s * rtb_lowpass_filter_est1[2];
      G[ipiv_tmp + 12] = s * rtb_lowpass_filter_est1[3];
      for (idx = 0; idx < 4; idx++) {
        b_kAcol = idx << 2;
        INDI_allocator_B.G1[ipiv_tmp + b_kAcol] =
          INDI_allocator_B.Memory3[b_kAcol + ipiv_tmp] - (((b[b_kAcol + 1] *
          G[ipiv_tmp + 4] + b[b_kAcol] * G[ipiv_tmp]) + b[b_kAcol + 2] *
          G[ipiv_tmp + 8]) + b[b_kAcol + 3] * G[ipiv_tmp + 12]);
      }
    }

    for (jA = 0; jA < 16; jA++) {
      if (INDI_allocator_B.G1[jA] > 30.0) {
        /* '<S5>:1:18' */
        INDI_allocator_B.G1[jA] = 30.0;
      }

      if (INDI_allocator_B.G1[jA] < -30.0) {
        /* '<S5>:1:19' */
        INDI_allocator_B.G1[jA] = -30.0;
      }
    }

    /* End of MATLAB Function: '<S1>/MATLAB Function2' */

    /* StateSpace: '<S1>/lowpass_filter1' */
    /* MATLAB Function 'INDI_allocator/FD': '<S2>:1' */
    /* '<S2>:1:5' */
    INDI_allocator_B.lowpass_filter1[0] = 0.0;
    INDI_allocator_B.lowpass_filter1[1] = 0.0;
    INDI_allocator_B.lowpass_filter1[2] = 0.0;
    INDI_allocator_B.lowpass_filter1[3] = 0.0;
    INDI_allocator_B.lowpass_filter1[0] += INDI_allocator_P.lowpass_filter1_C[0]
      * INDI_allocator_X.lowpass_filter1_CSTATE[0];
    INDI_allocator_B.lowpass_filter1[1] += INDI_allocator_P.lowpass_filter1_C[1]
      * INDI_allocator_X.lowpass_filter1_CSTATE[1];
    INDI_allocator_B.lowpass_filter1[2] += INDI_allocator_P.lowpass_filter1_C[2]
      * INDI_allocator_X.lowpass_filter1_CSTATE[2];
    INDI_allocator_B.lowpass_filter1[3] += INDI_allocator_P.lowpass_filter1_C[3]
      * INDI_allocator_X.lowpass_filter1_CSTATE[3];

    /* Sum: '<S1>/Sum of Elements' */
    INDI_allocator_B.SumofElements = ((INDI_allocator_B.lowpass_filter1[0] +
      INDI_allocator_B.lowpass_filter1[1]) + INDI_allocator_B.lowpass_filter1[2])
      + INDI_allocator_B.lowpass_filter1[3];
    if (rtmIsMajorTimeStep((&INDI_allocator_M)) &&
        (&INDI_allocator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Memory: '<S1>/Memory' */
      INDI_allocator_B.Memory[0] = INDI_allocator_DW.Memory_PreviousInput[0];
      INDI_allocator_B.Memory[1] = INDI_allocator_DW.Memory_PreviousInput[1];
      INDI_allocator_B.Memory[2] = INDI_allocator_DW.Memory_PreviousInput[2];
      INDI_allocator_B.Memory[3] = INDI_allocator_DW.Memory_PreviousInput[3];

      /* Memory: '<S1>/Memory2' */
      INDI_allocator_B.Memory2[0] = INDI_allocator_DW.Memory2_PreviousInput[0];
      INDI_allocator_B.Memory2[1] = INDI_allocator_DW.Memory2_PreviousInput[1];
      INDI_allocator_B.Memory2[2] = INDI_allocator_DW.Memory2_PreviousInput[2];
      INDI_allocator_B.Memory2[3] = INDI_allocator_DW.Memory2_PreviousInput[3];

      /* Memory: '<S1>/Memory1' */
      memcpy(&INDI_allocator_B.Memory1[0],
             &INDI_allocator_DW.Memory1_PreviousInput[0], sizeof(real_T) << 4U);
    }

    if (rtmIsMajorTimeStep((&INDI_allocator_M)) &&
        (&INDI_allocator_M)->Timing.TaskCounters.TID[2] == 0) {
      /* SignalConversion: '<S3>/TmpSignal ConversionAt SFunction Inport4' incorporates:
       *  Inport: '<Root>/pqr_dot_cmd'
       *  MATLAB Function: '<S1>/MATLAB Function'
       */
      rtb_TmpSignalConversionAtSFunct[0] = INDI_allocator_U.u_pqr[0];
      rtb_TmpSignalConversionAtSFunct[1] = INDI_allocator_U.u_pqr[1];
      rtb_TmpSignalConversionAtSFunct[2] = INDI_allocator_U.u_pqr[2];

      /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
       *  Inport: '<Root>/thrust_cmd'
       */
      /* MATLAB Function 'INDI_allocator/MATLAB Function': '<S3>:1' */
      /* '<S3>:1:38' */
      /* '<S3>:1:37' */
      /* '<S3>:1:27' */
      /* '<S3>:1:24' */
      /* '<S3>:1:18' */
      /* '<S3>:1:8' */
      /* '<S3>:1:9' */
      /* '<S3>:1:11' */
      rtb_TmpSignalConversionAtSFunct[3] = (INDI_allocator_U.thrust_cmd * 956.0
        + 300.0) * 4.0;

      /* '<S3>:1:18' */
      /* '<S3>:1:23' */
      /* '<S3>:1:24' */
      memcpy(&G[0], &g[0], sizeof(real_T) << 4U);
      memcpy(&G2[0], &h[0], sizeof(real_T) << 4U);
      G[2] = INDI_allocator_P.para[4] * -0.25;
      G[6] = INDI_allocator_P.para[4] * 0.25;
      G[10] = INDI_allocator_P.para[4] * -0.25;
      G[14] = INDI_allocator_P.para[4] * 0.25;

      /* '<S3>:1:37' */
      /* '<S3>:1:38' */
      G2[2] = INDI_allocator_P.para[4] * -0.0612093;
      G2[6] = INDI_allocator_P.para[4] * 0.065367000000000008;
      G2[10] = INDI_allocator_P.para[4] * -0.0657419;
      G2[14] = INDI_allocator_P.para[4] * 0.0654516;

      /* '<S3>:1:39' */
      /* '<S3>:1:40' */
      for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
        G[ipiv_tmp] = G[ipiv_tmp] / 1000.0 + G2[ipiv_tmp];
      }

      ii_data[0] = 1;
      ii_data[1] = 2;
      ii_data[2] = 3;
      for (idx = 0; idx < 3; idx++) {
        b_kAcol = idx * 5;
        jA = 0;
        ix = b_kAcol;
        smax = std::abs(G[b_kAcol]);
        for (ipiv_tmp = 2; ipiv_tmp <= 4 - idx; ipiv_tmp++) {
          ix++;
          s = std::abs(G[ix]);
          if (s > smax) {
            jA = ipiv_tmp - 1;
            smax = s;
          }
        }

        if (G[b_kAcol + jA] != 0.0) {
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

          ix = (b_kAcol - idx) + 4;
          for (jA = b_kAcol + 1; jA < ix; jA++) {
            G[jA] /= G[b_kAcol];
          }
        }

        jA = b_kAcol + 5;
        ix = b_kAcol + 4;
        for (ipiv_tmp = 0; ipiv_tmp <= 2 - idx; ipiv_tmp++) {
          smax = G[ix];
          if (G[ix] != 0.0) {
            c_ix = b_kAcol + 1;
            d = (jA - idx) + 3;
            for (ijA = jA; ijA < d; ijA++) {
              G[ijA] += G[c_ix] * -smax;
              c_ix++;
            }
          }

          ix += 4;
          jA += 4;
        }

        tmp[idx] = INDI_allocator_B.Derivative[idx];
      }

      tmp[3] = INDI_allocator_B.SumofElements;
      for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
        INDI_allocator_B.du[ipiv_tmp] = (((G2[ipiv_tmp + 4] *
          INDI_allocator_B.Memory[1] + G2[ipiv_tmp] * INDI_allocator_B.Memory[0])
          + G2[ipiv_tmp + 8] * INDI_allocator_B.Memory[2]) + G2[ipiv_tmp + 12] *
          INDI_allocator_B.Memory[3]) +
          (rtb_TmpSignalConversionAtSFunct[ipiv_tmp] - tmp[ipiv_tmp]);
      }

      if (ii_data[0] != 1) {
        smax = INDI_allocator_B.du[0];
        b_kAcol = ii_data[0] - 1;
        INDI_allocator_B.du[0] = INDI_allocator_B.du[b_kAcol];
        INDI_allocator_B.du[b_kAcol] = smax;
      }

      if (ii_data[1] != 2) {
        smax = INDI_allocator_B.du[1];
        b_kAcol = ii_data[1] - 1;
        INDI_allocator_B.du[1] = INDI_allocator_B.du[b_kAcol];
        INDI_allocator_B.du[b_kAcol] = smax;
      }

      if (ii_data[2] != 3) {
        smax = INDI_allocator_B.du[2];
        b_kAcol = ii_data[2] - 1;
        INDI_allocator_B.du[2] = INDI_allocator_B.du[b_kAcol];
        INDI_allocator_B.du[b_kAcol] = smax;
      }

      if (INDI_allocator_B.du[0] != 0.0) {
        for (idx = 1; idx + 1 < 5; idx++) {
          INDI_allocator_B.du[idx] -= INDI_allocator_B.du[0] * G[idx];
        }
      }

      if (INDI_allocator_B.du[1] != 0.0) {
        for (idx = 2; idx + 1 < 5; idx++) {
          INDI_allocator_B.du[idx] -= G[idx + 4] * INDI_allocator_B.du[1];
        }
      }

      if (INDI_allocator_B.du[2] != 0.0) {
        for (idx = 3; idx + 1 < 5; idx++) {
          INDI_allocator_B.du[idx] -= G[idx + 8] * INDI_allocator_B.du[2];
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

      b_kAcol = 0;
      if (INDI_allocator_B.du[0] != 0.0) {
        INDI_allocator_B.du[0] /= G[0];
      }

      INDI_allocator_B.du[0] = INDI_allocator_B.du[0] * 2.0 * 3.1415926535897931
        / 60.0;
      INDI_allocator_B.du[1] = INDI_allocator_B.du[1] * 2.0 * 3.1415926535897931
        / 60.0;
      INDI_allocator_B.du[2] = INDI_allocator_B.du[2] * 2.0 * 3.1415926535897931
        / 60.0;
      INDI_allocator_B.du[3] = INDI_allocator_B.du[3] * 2.0 * 3.1415926535897931
        / 60.0;

      /* '<S3>:1:43' */
      INDI_allocator_B.w_cmd[0] = INDI_allocator_B.lowpass_filter1[0] +
        INDI_allocator_B.du[0];
      INDI_allocator_B.w_cmd[1] = INDI_allocator_B.lowpass_filter1[1] +
        INDI_allocator_B.du[1];
      INDI_allocator_B.w_cmd[2] = INDI_allocator_B.lowpass_filter1[2] +
        INDI_allocator_B.du[2];
      INDI_allocator_B.w_cmd[3] = INDI_allocator_B.lowpass_filter1[3] +
        INDI_allocator_B.du[3];

      /* '<S3>:1:45' */
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (b_kAcol < 4)) {
        if (INDI_allocator_B.w_cmd[b_kAcol] >= 1256.0) {
          idx++;
          ii_data[idx - 1] = (int8_T)(b_kAcol + 1);
          if (idx >= 4) {
            exitg1 = true;
          } else {
            b_kAcol++;
          }
        } else {
          b_kAcol++;
        }
      }

      if (1 > idx) {
        idx = 0;
      }

      ix = idx;
      if (0 <= idx - 1) {
        memcpy(&i_up_data[0], &ii_data[0], idx * sizeof(int8_T));
      }

      /* '<S3>:1:46' */
      idx = 0;
      b_kAcol = 0;
      exitg1 = false;
      while ((!exitg1) && (b_kAcol < 4)) {
        if (INDI_allocator_B.w_cmd[b_kAcol] < 300.0) {
          idx++;
          ii_data[idx - 1] = (int8_T)(b_kAcol + 1);
          if (idx >= 4) {
            exitg1 = true;
          } else {
            b_kAcol++;
          }
        } else {
          b_kAcol++;
        }
      }

      if (1 > idx) {
        idx = 0;
      }

      /* '<S3>:1:48' */
      for (ipiv_tmp = 0; ipiv_tmp < ix; ipiv_tmp++) {
        b_kAcol = i_up_data[ipiv_tmp] - 1;
        INDI_allocator_B.du[b_kAcol] = 1256.0 -
          INDI_allocator_B.lowpass_filter1[b_kAcol];
      }

      /* '<S3>:1:49' */
      for (ipiv_tmp = 0; ipiv_tmp < idx; ipiv_tmp++) {
        b_kAcol = ii_data[ipiv_tmp] - 1;
        INDI_allocator_B.du[b_kAcol] = 300.0 -
          INDI_allocator_B.lowpass_filter1[b_kAcol];
      }

      /* '<S3>:1:51' */
      for (ipiv_tmp = 0; ipiv_tmp < ix; ipiv_tmp++) {
        INDI_allocator_B.w_cmd[i_up_data[ipiv_tmp] - 1] = 1256.0;
      }

      /* '<S3>:1:52' */
      for (ipiv_tmp = 0; ipiv_tmp < idx; ipiv_tmp++) {
        INDI_allocator_B.w_cmd[ii_data[ipiv_tmp] - 1] = 300.0;
      }

      /* '<S3>:1:55' */
      rtb_TmpSignalConversionAtSFunct[0] = (INDI_allocator_B.w_cmd[0] - 300.0) *
        2.0 / 956.0 + -1.0;
      rtb_TmpSignalConversionAtSFunct[1] = (INDI_allocator_B.w_cmd[1] - 300.0) *
        2.0 / 956.0 + -1.0;
      rtb_TmpSignalConversionAtSFunct[2] = (INDI_allocator_B.w_cmd[2] - 300.0) *
        2.0 / 956.0 + -1.0;
      rtb_TmpSignalConversionAtSFunct[3] = (INDI_allocator_B.w_cmd[3] - 300.0) *
        2.0 / 956.0 + -1.0;
    }

    /* MATLAB Function: '<S1>/MATLAB Function1' */
    /* MATLAB Function 'INDI_allocator/MATLAB Function1': '<S4>:1' */
    /* '<S4>:1:8' */
    rtb_lowpass_filter_est1[0] = rtb_lowpass_filter_est1[0] * 2.0 *
      3.1415926535897931 / 60.0;
    rtb_lowpass_filter_est1[1] = rtb_lowpass_filter_est1[1] * 2.0 *
      3.1415926535897931 / 60.0;
    rtb_lowpass_filter_est1[2] = rtb_lowpass_filter_est1[2] * 2.0 *
      3.1415926535897931 / 60.0;
    smax = rtb_lowpass_filter_est1[3] * 2.0 * 3.1415926535897931 / 60.0;
    rtb_lowpass_filter_est1[3] = smax;
    rtb_lowpass_filter_est1_0 = 0.0;
    s = rtb_Derivative2[0] - (((rtb_lowpass_filter_est1[0] *
      INDI_allocator_B.Memory2[0] + rtb_lowpass_filter_est1[1] *
      INDI_allocator_B.Memory2[1]) + rtb_lowpass_filter_est1[2] *
      INDI_allocator_B.Memory2[2]) + smax * INDI_allocator_B.Memory2[3]);
    rtb_lowpass_filter_est1_1 = 0.0;
    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      idx = ipiv_tmp << 2;
      rtb_lowpass_filter_est1_2 = INDI_allocator_B.Memory1[idx + 3] * smax +
        (INDI_allocator_B.Memory1[idx + 2] * rtb_lowpass_filter_est1[2] +
         (INDI_allocator_B.Memory1[idx + 1] * rtb_lowpass_filter_est1[1] +
          INDI_allocator_B.Memory1[idx] * rtb_lowpass_filter_est1[0]));
      rtb_lowpass_filter_est1_0 += rtb_lowpass_filter_est1_2 *
        rtb_lowpass_filter_est1[ipiv_tmp];
      rtb_lowpass_filter_est1_2 = INDI_allocator_B.Memory1[(ipiv_tmp << 2) + 3] *
        smax + (INDI_allocator_B.Memory1[(ipiv_tmp << 2) + 2] *
                rtb_lowpass_filter_est1[2] + (INDI_allocator_B.Memory1[(ipiv_tmp
                  << 2) + 1] * rtb_lowpass_filter_est1[1] +
                 INDI_allocator_B.Memory1[ipiv_tmp << 2] *
                 rtb_lowpass_filter_est1[0]));
      rtb_lowpass_filter_est1_1 += rtb_lowpass_filter_est1_2 *
        rtb_lowpass_filter_est1[ipiv_tmp];
      G[ipiv_tmp] = rtb_lowpass_filter_est1[ipiv_tmp] * rtb_lowpass_filter_est1
        [0];
      G[ipiv_tmp + 4] = rtb_lowpass_filter_est1[ipiv_tmp] *
        rtb_lowpass_filter_est1[1];
      G[ipiv_tmp + 8] = rtb_lowpass_filter_est1[ipiv_tmp] *
        rtb_lowpass_filter_est1[2];
      G[ipiv_tmp + 12] = rtb_lowpass_filter_est1[ipiv_tmp] * smax;
    }

    for (ipiv_tmp = 0; ipiv_tmp < 4; ipiv_tmp++) {
      for (idx = 0; idx < 4; idx++) {
        b_kAcol = idx << 2;
        ix = ipiv_tmp + b_kAcol;
        G2[ix] = 0.0;
        jA = b_kAcol + ipiv_tmp;
        G2[ix] = G2[jA] + G[b_kAcol] * INDI_allocator_B.Memory1[ipiv_tmp];
        G2[ix] = G[b_kAcol + 1] * INDI_allocator_B.Memory1[ipiv_tmp + 4] + G2[jA];
        G2[ix] = G[b_kAcol + 2] * INDI_allocator_B.Memory1[ipiv_tmp + 8] + G2[jA];
        G2[ix] = G[b_kAcol + 3] * INDI_allocator_B.Memory1[ipiv_tmp + 12] +
          G2[jA];
      }

      for (idx = 0; idx < 4; idx++) {
        b_kAcol = idx << 2;
        INDI_allocator_B.P_out[ipiv_tmp + b_kAcol] =
          INDI_allocator_B.Memory1[b_kAcol + ipiv_tmp] -
          (((INDI_allocator_B.Memory1[b_kAcol + 1] * G2[ipiv_tmp + 4] +
             INDI_allocator_B.Memory1[b_kAcol] * G2[ipiv_tmp]) +
            INDI_allocator_B.Memory1[b_kAcol + 2] * G2[ipiv_tmp + 8]) +
           INDI_allocator_B.Memory1[b_kAcol + 3] * G2[ipiv_tmp + 12]) / (1.0 +
          rtb_lowpass_filter_est1_1);
      }
    }

    /* '<S4>:1:13' */
    /* '<S4>:1:19' */
    for (ipiv_tmp = 0; ipiv_tmp < 16; ipiv_tmp++) {
      G2[ipiv_tmp] = INDI_allocator_B.Memory1[ipiv_tmp] / (1.0 +
        rtb_lowpass_filter_est1_0);
    }

    /* SignalConversion: '<S1>/TmpSignal ConversionAtlowpass_filter_est Inport1' incorporates:
     *  Inport: '<Root>/Az'
     */
    /* '<S4>:1:22' */
    /* '<S4>:1:24' */
    INDI_allocator_B.TmpSignalConversionAtlowpass_fi[0] =
      INDI_allocator_B.Derivative[0];
    INDI_allocator_B.TmpSignalConversionAtlowpass_fi[1] =
      INDI_allocator_B.Derivative[1];
    INDI_allocator_B.TmpSignalConversionAtlowpass_fi[2] =
      INDI_allocator_B.Derivative[2];
    INDI_allocator_B.TmpSignalConversionAtlowpass_fi[3] = INDI_allocator_U.Az;
    for (jA = 0; jA < 4; jA++) {
      /* MATLAB Function: '<S1>/MATLAB Function1' */
      rtb_lowpass_filter_est1_0 = G2[jA + 12] * smax + (G2[jA + 8] *
        rtb_lowpass_filter_est1[2] + (G2[jA + 4] * rtb_lowpass_filter_est1[1] +
        G2[jA] * rtb_lowpass_filter_est1[0]));
      INDI_allocator_B.G_out[jA] = rtb_lowpass_filter_est1_0 * s +
        INDI_allocator_B.Memory2[jA];

      /* StateSpace: '<S1>/actuator_dynamics1' */
      INDI_allocator_B.w[jA] = 0.0;

      /* StateSpace: '<S1>/lowpass_filter_actuator' */
      INDI_allocator_B.lowpass_filter_actuator[jA] = 0.0;

      /* StateSpace: '<S1>/lowpass_filter_indi' */
      INDI_allocator_B.lowpass_filter_indi[jA] = 0.0;

      /* StateSpace: '<S1>/actuator_dynamics1' */
      INDI_allocator_B.w[jA] += INDI_allocator_P.actuator_dynamics1_C[jA] *
        INDI_allocator_X.actuator_dynamics1_CSTATE[0];

      /* StateSpace: '<S1>/lowpass_filter_actuator' */
      INDI_allocator_B.lowpass_filter_actuator[jA] +=
        INDI_allocator_P.lowpass_filter_actuator_C[jA] *
        INDI_allocator_X.lowpass_filter_actuator_CSTATE[0];

      /* StateSpace: '<S1>/lowpass_filter_indi' */
      INDI_allocator_B.lowpass_filter_indi[jA] +=
        INDI_allocator_P.lowpass_filter_indi_C[jA] *
        INDI_allocator_X.lowpass_filter_indi_CSTATE[0];

      /* StateSpace: '<S1>/actuator_dynamics1' */
      INDI_allocator_B.w[jA] += INDI_allocator_P.actuator_dynamics1_C[4 + jA] *
        INDI_allocator_X.actuator_dynamics1_CSTATE[1];

      /* StateSpace: '<S1>/lowpass_filter_actuator' */
      INDI_allocator_B.lowpass_filter_actuator[jA] +=
        INDI_allocator_P.lowpass_filter_actuator_C[4 + jA] *
        INDI_allocator_X.lowpass_filter_actuator_CSTATE[1];

      /* StateSpace: '<S1>/lowpass_filter_indi' */
      INDI_allocator_B.lowpass_filter_indi[jA] +=
        INDI_allocator_P.lowpass_filter_indi_C[4 + jA] *
        INDI_allocator_X.lowpass_filter_indi_CSTATE[1];

      /* StateSpace: '<S1>/actuator_dynamics1' */
      INDI_allocator_B.w[jA] += INDI_allocator_P.actuator_dynamics1_C[8 + jA] *
        INDI_allocator_X.actuator_dynamics1_CSTATE[2];

      /* StateSpace: '<S1>/lowpass_filter_actuator' */
      INDI_allocator_B.lowpass_filter_actuator[jA] +=
        INDI_allocator_P.lowpass_filter_actuator_C[8 + jA] *
        INDI_allocator_X.lowpass_filter_actuator_CSTATE[2];

      /* StateSpace: '<S1>/lowpass_filter_indi' */
      INDI_allocator_B.lowpass_filter_indi[jA] +=
        INDI_allocator_P.lowpass_filter_indi_C[8 + jA] *
        INDI_allocator_X.lowpass_filter_indi_CSTATE[2];

      /* StateSpace: '<S1>/actuator_dynamics1' */
      INDI_allocator_B.w[jA] += INDI_allocator_P.actuator_dynamics1_C[12 + jA] *
        INDI_allocator_X.actuator_dynamics1_CSTATE[3];

      /* StateSpace: '<S1>/lowpass_filter_actuator' */
      INDI_allocator_B.lowpass_filter_actuator[jA] +=
        INDI_allocator_P.lowpass_filter_actuator_C[12 + jA] *
        INDI_allocator_X.lowpass_filter_actuator_CSTATE[3];

      /* StateSpace: '<S1>/lowpass_filter_indi' */
      INDI_allocator_B.lowpass_filter_indi[jA] +=
        INDI_allocator_P.lowpass_filter_indi_C[12 + jA] *
        INDI_allocator_X.lowpass_filter_indi_CSTATE[3];
    }

    /* End of Outputs for SubSystem: '<Root>/INDI_allocator' */
    if (rtmIsMajorTimeStep((&INDI_allocator_M)) &&
        (&INDI_allocator_M)->Timing.TaskCounters.TID[2] == 0) {
      /* Outport: '<Root>/w_cmd' */
      INDI_allocator_Y.w_cmd[0] = INDI_allocator_B.w_cmd[0];

      /* Outport: '<Root>/w_cmd_px4' */
      INDI_allocator_Y.w_cmd_px4[0] = rtb_TmpSignalConversionAtSFunct[0];

      /* Outport: '<Root>/w_cmd' */
      INDI_allocator_Y.w_cmd[1] = INDI_allocator_B.w_cmd[1];

      /* Outport: '<Root>/w_cmd_px4' */
      INDI_allocator_Y.w_cmd_px4[1] = rtb_TmpSignalConversionAtSFunct[1];

      /* Outport: '<Root>/w_cmd' */
      INDI_allocator_Y.w_cmd[2] = INDI_allocator_B.w_cmd[2];

      /* Outport: '<Root>/w_cmd_px4' */
      INDI_allocator_Y.w_cmd_px4[2] = rtb_TmpSignalConversionAtSFunct[2];

      /* Outport: '<Root>/w_cmd' */
      INDI_allocator_Y.w_cmd[3] = INDI_allocator_B.w_cmd[3];

      /* Outport: '<Root>/w_cmd_px4' */
      INDI_allocator_Y.w_cmd_px4[3] = rtb_TmpSignalConversionAtSFunct[3];
    }
  }

  if (rtmIsMajorTimeStep((&INDI_allocator_M))) {
    real_T (*lastU)[3];
    real_T (*lastU_0)[4];

    /* Update for Atomic SubSystem: '<Root>/INDI_allocator' */
    /* Update for Derivative: '<S1>/Derivative' */
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

    /* End of Update for Derivative: '<S1>/Derivative' */

    /* Update for Derivative: '<S1>/Derivative2' */
    if (INDI_allocator_DW.TimeStampA_n == (rtInf)) {
      INDI_allocator_DW.TimeStampA_n = (&INDI_allocator_M)->Timing.t[0];
      lastU_0 = &INDI_allocator_DW.LastUAtTimeA_c;
    } else if (INDI_allocator_DW.TimeStampB_m == (rtInf)) {
      INDI_allocator_DW.TimeStampB_m = (&INDI_allocator_M)->Timing.t[0];
      lastU_0 = &INDI_allocator_DW.LastUAtTimeB_p;
    } else if (INDI_allocator_DW.TimeStampA_n < INDI_allocator_DW.TimeStampB_m)
    {
      INDI_allocator_DW.TimeStampA_n = (&INDI_allocator_M)->Timing.t[0];
      lastU_0 = &INDI_allocator_DW.LastUAtTimeA_c;
    } else {
      INDI_allocator_DW.TimeStampB_m = (&INDI_allocator_M)->Timing.t[0];
      lastU_0 = &INDI_allocator_DW.LastUAtTimeB_p;
    }

    (*lastU_0)[0] = INDI_allocator_B.lowpass_filter_est[0];
    (*lastU_0)[1] = INDI_allocator_B.lowpass_filter_est[1];
    (*lastU_0)[2] = INDI_allocator_B.lowpass_filter_est[2];
    (*lastU_0)[3] = INDI_allocator_B.lowpass_filter_est[3];

    /* End of Update for Derivative: '<S1>/Derivative2' */
    if (rtmIsMajorTimeStep((&INDI_allocator_M)) &&
        (&INDI_allocator_M)->Timing.TaskCounters.TID[1] == 0) {
      /* Update for Memory: '<S1>/Memory' */
      INDI_allocator_DW.Memory_PreviousInput[0] = INDI_allocator_B.du[0];

      /* Update for Memory: '<S1>/Memory2' */
      INDI_allocator_DW.Memory2_PreviousInput[0] = INDI_allocator_B.G_out[0];

      /* Update for Memory: '<S1>/Memory' */
      INDI_allocator_DW.Memory_PreviousInput[1] = INDI_allocator_B.du[1];

      /* Update for Memory: '<S1>/Memory2' */
      INDI_allocator_DW.Memory2_PreviousInput[1] = INDI_allocator_B.G_out[1];

      /* Update for Memory: '<S1>/Memory' */
      INDI_allocator_DW.Memory_PreviousInput[2] = INDI_allocator_B.du[2];

      /* Update for Memory: '<S1>/Memory2' */
      INDI_allocator_DW.Memory2_PreviousInput[2] = INDI_allocator_B.G_out[2];

      /* Update for Memory: '<S1>/Memory' */
      INDI_allocator_DW.Memory_PreviousInput[3] = INDI_allocator_B.du[3];

      /* Update for Memory: '<S1>/Memory2' */
      INDI_allocator_DW.Memory2_PreviousInput[3] = INDI_allocator_B.G_out[3];

      /* Update for Memory: '<S1>/Memory3' */
      memcpy(&INDI_allocator_DW.Memory3_PreviousInput[0], &INDI_allocator_B.G1[0],
             sizeof(real_T) << 4U);

      /* Update for Memory: '<S1>/Memory1' */
      memcpy(&INDI_allocator_DW.Memory1_PreviousInput[0],
             &INDI_allocator_B.P_out[0], sizeof(real_T) << 4U);
    }

    /* End of Update for SubSystem: '<Root>/INDI_allocator' */
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

    rate_scheduler((&INDI_allocator_M));
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void INDI_allocatorModelClass::INDI_allocator_derivatives()
{
  int_T is;
  real_T lowpass_filter_CSTATE_tmp;
  real_T lowpass_filter_CSTATE_tmp_0;
  real_T lowpass_filter_est_CSTATE_tmp;
  real_T lowpass_filter_est_CSTATE_tmp_0;
  real_T lowpass_filter_est_CSTATE_tmp_1;
  real_T lowpass_filter_est_CSTATE_tmp_2;
  real_T lowpass_filter_est_CSTATE_tmp_3;
  real_T lowpass_filter_est_CSTATE_tmp_4;
  real_T lowpass_filter_est_CSTATE_tmp_5;
  real_T lowpass_filter_est_CSTATE_tmp_6;
  real_T lowpass_filter_est_CSTATE_tmp_7;
  XDot_INDI_allocator_T *_rtXdot;
  _rtXdot = ((XDot_INDI_allocator_T *) (&INDI_allocator_M)->derivs);

  /* Derivatives for Atomic SubSystem: '<Root>/INDI_allocator' */
  /* Derivatives for StateSpace: '<S1>/lowpass_filter ' incorporates:
   *  Inport: '<Root>/pqr'
   */
  for (is = 0; is < 3; is++) {
    _rtXdot->lowpass_filter_CSTATE[is] = 0.0;
    _rtXdot->lowpass_filter_CSTATE[is] += -1.0 / t_indi *
      INDI_allocator_ConstP.pooled2[is] *
      INDI_allocator_X.lowpass_filter_CSTATE[0];
    lowpass_filter_CSTATE_tmp = INDI_allocator_ConstP.pooled2[3 + is];
    _rtXdot->lowpass_filter_CSTATE[is] += -1.0 / t_indi *
      lowpass_filter_CSTATE_tmp * INDI_allocator_X.lowpass_filter_CSTATE[1];
    lowpass_filter_CSTATE_tmp_0 = INDI_allocator_ConstP.pooled2[6 + is];
    _rtXdot->lowpass_filter_CSTATE[is] += -1.0 / t_indi *
      lowpass_filter_CSTATE_tmp_0 * INDI_allocator_X.lowpass_filter_CSTATE[2];
    _rtXdot->lowpass_filter_CSTATE[is] += 1.0 / t_indi *
      INDI_allocator_ConstP.pooled2[is] * INDI_allocator_U.pqr[0];
    _rtXdot->lowpass_filter_CSTATE[is] += 1.0 / t_indi *
      lowpass_filter_CSTATE_tmp * INDI_allocator_U.pqr[1];
    _rtXdot->lowpass_filter_CSTATE[is] += 1.0 / t_indi *
      lowpass_filter_CSTATE_tmp_0 * INDI_allocator_U.pqr[2];
  }

  /* End of Derivatives for StateSpace: '<S1>/lowpass_filter ' */
  for (is = 0; is < 4; is++) {
    /* Derivatives for StateSpace: '<S1>/lowpass_filter_est ' incorporates:
     *  StateSpace: '<S1>/actuator_dynamics1'
     *  StateSpace: '<S1>/lowpass_filter_est 1'
     *  StateSpace: '<S1>/lowpass_filter_indi'
     */
    _rtXdot->lowpass_filter_est_CSTATE[is] = 0.0;
    lowpass_filter_est_CSTATE_tmp_0 = -1.0 / INDI_allocator_P.t_G_est *
      INDI_allocator_ConstP.pooled1[is];
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_0 *
      INDI_allocator_X.lowpass_filter_est_CSTATE[0];
    lowpass_filter_CSTATE_tmp = INDI_allocator_ConstP.pooled1[4 + is];
    lowpass_filter_est_CSTATE_tmp_1 = -1.0 / INDI_allocator_P.t_G_est *
      lowpass_filter_CSTATE_tmp;
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_1 *
      INDI_allocator_X.lowpass_filter_est_CSTATE[1];
    lowpass_filter_CSTATE_tmp_0 = INDI_allocator_ConstP.pooled1[8 + is];
    lowpass_filter_est_CSTATE_tmp_2 = -1.0 / INDI_allocator_P.t_G_est *
      lowpass_filter_CSTATE_tmp_0;
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_2 *
      INDI_allocator_X.lowpass_filter_est_CSTATE[2];
    lowpass_filter_est_CSTATE_tmp = INDI_allocator_ConstP.pooled1[12 + is];
    lowpass_filter_est_CSTATE_tmp_3 = -1.0 / INDI_allocator_P.t_G_est *
      lowpass_filter_est_CSTATE_tmp;
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_3 *
      INDI_allocator_X.lowpass_filter_est_CSTATE[3];
    lowpass_filter_est_CSTATE_tmp_4 = 1.0 / INDI_allocator_P.t_G_est *
      INDI_allocator_ConstP.pooled1[is];
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_4 *
      INDI_allocator_B.TmpSignalConversionAtlowpass_fi[0];
    lowpass_filter_est_CSTATE_tmp_5 = 1.0 / INDI_allocator_P.t_G_est *
      lowpass_filter_CSTATE_tmp;
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_5 *
      INDI_allocator_B.TmpSignalConversionAtlowpass_fi[1];
    lowpass_filter_est_CSTATE_tmp_6 = 1.0 / INDI_allocator_P.t_G_est *
      lowpass_filter_CSTATE_tmp_0;
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_6 *
      INDI_allocator_B.TmpSignalConversionAtlowpass_fi[2];
    lowpass_filter_est_CSTATE_tmp_7 = 1.0 / INDI_allocator_P.t_G_est *
      lowpass_filter_est_CSTATE_tmp;
    _rtXdot->lowpass_filter_est_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_7 *
      INDI_allocator_B.TmpSignalConversionAtlowpass_fi[3];

    /* Derivatives for StateSpace: '<S1>/lowpass_filter_est 1' */
    _rtXdot->lowpass_filter_est1_CSTATE[is] = 0.0;
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_0 *
      INDI_allocator_X.lowpass_filter_est1_CSTATE[0];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_1 *
      INDI_allocator_X.lowpass_filter_est1_CSTATE[1];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_2 *
      INDI_allocator_X.lowpass_filter_est1_CSTATE[2];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_3 *
      INDI_allocator_X.lowpass_filter_est1_CSTATE[3];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_4 *
      INDI_allocator_B.lowpass_filter_indi[0];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_5 *
      INDI_allocator_B.lowpass_filter_indi[1];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_6 *
      INDI_allocator_B.lowpass_filter_indi[2];
    _rtXdot->lowpass_filter_est1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_7 *
      INDI_allocator_B.lowpass_filter_indi[3];

    /* Derivatives for StateSpace: '<S1>/lowpass_filter1' */
    _rtXdot->lowpass_filter1_CSTATE[is] = 0.0;

    /* Derivatives for StateSpace: '<S1>/actuator_dynamics1' incorporates:
     *  StateSpace: '<S1>/lowpass_filter_actuator'
     */
    _rtXdot->actuator_dynamics1_CSTATE[is] = 0.0;
    lowpass_filter_est_CSTATE_tmp_0 = -1.0 / t_w *
      INDI_allocator_ConstP.pooled1[is];
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_0 *
      INDI_allocator_X.actuator_dynamics1_CSTATE[0];
    lowpass_filter_est_CSTATE_tmp_1 = -1.0 / t_w * lowpass_filter_CSTATE_tmp;
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_1 *
      INDI_allocator_X.actuator_dynamics1_CSTATE[1];
    lowpass_filter_est_CSTATE_tmp_2 = -1.0 / t_w * lowpass_filter_CSTATE_tmp_0;
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_2 *
      INDI_allocator_X.actuator_dynamics1_CSTATE[2];
    lowpass_filter_est_CSTATE_tmp_3 = -1.0 / t_w * lowpass_filter_est_CSTATE_tmp;
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_3 *
      INDI_allocator_X.actuator_dynamics1_CSTATE[3];
    lowpass_filter_est_CSTATE_tmp_4 = 1.0 / t_w *
      INDI_allocator_ConstP.pooled1[is];
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_4 *
      INDI_allocator_B.w_cmd[0];
    lowpass_filter_est_CSTATE_tmp_5 = 1.0 / t_w * lowpass_filter_CSTATE_tmp;
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_5 *
      INDI_allocator_B.w_cmd[1];
    lowpass_filter_est_CSTATE_tmp_6 = 1.0 / t_w * lowpass_filter_CSTATE_tmp_0;
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_6 *
      INDI_allocator_B.w_cmd[2];
    lowpass_filter_est_CSTATE_tmp_7 = 1.0 / t_w * lowpass_filter_est_CSTATE_tmp;
    _rtXdot->actuator_dynamics1_CSTATE[is] += lowpass_filter_est_CSTATE_tmp_7 *
      INDI_allocator_B.w_cmd[3];

    /* Derivatives for StateSpace: '<S1>/lowpass_filter_actuator' */
    _rtXdot->lowpass_filter_actuator_CSTATE[is] = 0.0;
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_0 *
      INDI_allocator_X.lowpass_filter_actuator_CSTATE[0];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_1 *
      INDI_allocator_X.lowpass_filter_actuator_CSTATE[1];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_2 *
      INDI_allocator_X.lowpass_filter_actuator_CSTATE[2];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_3 *
      INDI_allocator_X.lowpass_filter_actuator_CSTATE[3];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_4 * INDI_allocator_B.du[0];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_5 * INDI_allocator_B.du[1];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_6 * INDI_allocator_B.du[2];
    _rtXdot->lowpass_filter_actuator_CSTATE[is] +=
      lowpass_filter_est_CSTATE_tmp_7 * INDI_allocator_B.du[3];

    /* Derivatives for StateSpace: '<S1>/lowpass_filter_indi' */
    _rtXdot->lowpass_filter_indi_CSTATE[is] = 0.0;
    _rtXdot->lowpass_filter_indi_CSTATE[is] += -1.0 / t_indi *
      INDI_allocator_ConstP.pooled1[is] *
      INDI_allocator_X.lowpass_filter_indi_CSTATE[0];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += -1.0 / t_indi *
      lowpass_filter_CSTATE_tmp * INDI_allocator_X.lowpass_filter_indi_CSTATE[1];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += -1.0 / t_indi *
      lowpass_filter_CSTATE_tmp_0 * INDI_allocator_X.lowpass_filter_indi_CSTATE
      [2];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += -1.0 / t_indi *
      lowpass_filter_est_CSTATE_tmp *
      INDI_allocator_X.lowpass_filter_indi_CSTATE[3];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += 1.0 / t_indi *
      INDI_allocator_ConstP.pooled1[is] *
      INDI_allocator_B.lowpass_filter_actuator[0];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += 1.0 / t_indi *
      lowpass_filter_CSTATE_tmp * INDI_allocator_B.lowpass_filter_actuator[1];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += 1.0 / t_indi *
      lowpass_filter_CSTATE_tmp_0 * INDI_allocator_B.lowpass_filter_actuator[2];
    _rtXdot->lowpass_filter_indi_CSTATE[is] += 1.0 / t_indi *
      lowpass_filter_est_CSTATE_tmp * INDI_allocator_B.lowpass_filter_actuator[3];
  }

  /* Derivatives for StateSpace: '<S1>/lowpass_filter1' */
  _rtXdot->lowpass_filter1_CSTATE[0] += INDI_allocator_P.lowpass_filter1_A[0] *
    INDI_allocator_X.lowpass_filter1_CSTATE[0];
  _rtXdot->lowpass_filter1_CSTATE[1] += INDI_allocator_P.lowpass_filter1_A[1] *
    INDI_allocator_X.lowpass_filter1_CSTATE[1];
  _rtXdot->lowpass_filter1_CSTATE[2] += INDI_allocator_P.lowpass_filter1_A[2] *
    INDI_allocator_X.lowpass_filter1_CSTATE[2];
  _rtXdot->lowpass_filter1_CSTATE[3] += INDI_allocator_P.lowpass_filter1_A[3] *
    INDI_allocator_X.lowpass_filter1_CSTATE[3];
  _rtXdot->lowpass_filter1_CSTATE[0] += INDI_allocator_P.lowpass_filter1_B[0] *
    INDI_allocator_B.w[0];
  _rtXdot->lowpass_filter1_CSTATE[1] += INDI_allocator_P.lowpass_filter1_B[1] *
    INDI_allocator_B.w[1];
  _rtXdot->lowpass_filter1_CSTATE[2] += INDI_allocator_P.lowpass_filter1_B[2] *
    INDI_allocator_B.w[2];
  _rtXdot->lowpass_filter1_CSTATE[3] += INDI_allocator_P.lowpass_filter1_B[3] *
    INDI_allocator_B.w[3];

  /* End of Derivatives for SubSystem: '<Root>/INDI_allocator' */
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
  (void) memset((void *)&INDI_allocator_Y, 0,
                sizeof(ExtY_INDI_allocator_T));

  /* SystemInitialize for Atomic SubSystem: '<Root>/INDI_allocator' */
  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter ' */
  INDI_allocator_X.lowpass_filter_CSTATE[0] =
    INDI_allocator_P.lowpass_filter_InitialCondition;
  INDI_allocator_X.lowpass_filter_CSTATE[1] =
    INDI_allocator_P.lowpass_filter_InitialCondition;
  INDI_allocator_X.lowpass_filter_CSTATE[2] =
    INDI_allocator_P.lowpass_filter_InitialCondition;

  /* InitializeConditions for Derivative: '<S1>/Derivative' */
  INDI_allocator_DW.TimeStampA = (rtInf);
  INDI_allocator_DW.TimeStampB = (rtInf);

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_est ' */
  INDI_allocator_X.lowpass_filter_est_CSTATE[0] =
    INDI_allocator_P.lowpass_filter_est_InitialCondi;
  INDI_allocator_X.lowpass_filter_est_CSTATE[1] =
    INDI_allocator_P.lowpass_filter_est_InitialCondi;
  INDI_allocator_X.lowpass_filter_est_CSTATE[2] =
    INDI_allocator_P.lowpass_filter_est_InitialCondi;
  INDI_allocator_X.lowpass_filter_est_CSTATE[3] =
    INDI_allocator_P.lowpass_filter_est_InitialCondi;

  /* InitializeConditions for Derivative: '<S1>/Derivative2' */
  INDI_allocator_DW.TimeStampA_n = (rtInf);
  INDI_allocator_DW.TimeStampB_m = (rtInf);

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_est 1' */
  INDI_allocator_X.lowpass_filter_est1_CSTATE[0] =
    INDI_allocator_P.lowpass_filter_est1_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter1' */
  INDI_allocator_X.lowpass_filter1_CSTATE[0] = INDI_allocator_P.w0[0];

  /* InitializeConditions for Memory: '<S1>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[0] =
    INDI_allocator_P.Memory_InitialCondition[0];

  /* InitializeConditions for Memory: '<S1>/Memory2' */
  INDI_allocator_DW.Memory2_PreviousInput[0] = INDI_allocator_P.G_init[0];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_est 1' */
  INDI_allocator_X.lowpass_filter_est1_CSTATE[1] =
    INDI_allocator_P.lowpass_filter_est1_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter1' */
  INDI_allocator_X.lowpass_filter1_CSTATE[1] = INDI_allocator_P.w0[1];

  /* InitializeConditions for Memory: '<S1>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[1] =
    INDI_allocator_P.Memory_InitialCondition[1];

  /* InitializeConditions for Memory: '<S1>/Memory2' */
  INDI_allocator_DW.Memory2_PreviousInput[1] = INDI_allocator_P.G_init[1];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_est 1' */
  INDI_allocator_X.lowpass_filter_est1_CSTATE[2] =
    INDI_allocator_P.lowpass_filter_est1_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter1' */
  INDI_allocator_X.lowpass_filter1_CSTATE[2] = INDI_allocator_P.w0[2];

  /* InitializeConditions for Memory: '<S1>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[2] =
    INDI_allocator_P.Memory_InitialCondition[2];

  /* InitializeConditions for Memory: '<S1>/Memory2' */
  INDI_allocator_DW.Memory2_PreviousInput[2] = INDI_allocator_P.G_init[2];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_est 1' */
  INDI_allocator_X.lowpass_filter_est1_CSTATE[3] =
    INDI_allocator_P.lowpass_filter_est1_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter1' */
  INDI_allocator_X.lowpass_filter1_CSTATE[3] = INDI_allocator_P.w0[3];

  /* InitializeConditions for Memory: '<S1>/Memory' */
  INDI_allocator_DW.Memory_PreviousInput[3] =
    INDI_allocator_P.Memory_InitialCondition[3];

  /* InitializeConditions for Memory: '<S1>/Memory2' */
  INDI_allocator_DW.Memory2_PreviousInput[3] = INDI_allocator_P.G_init[3];

  /* InitializeConditions for Memory: '<S1>/Memory3' */
  memcpy(&INDI_allocator_DW.Memory3_PreviousInput[0],
         &INDI_allocator_P.G_init_lms[0], sizeof(real_T) << 4U);

  /* InitializeConditions for Memory: '<S1>/Memory1' */
  memcpy(&INDI_allocator_DW.Memory1_PreviousInput[0],
         &INDI_allocator_P.P_G_init[0], sizeof(real_T) << 4U);

  /* InitializeConditions for StateSpace: '<S1>/actuator_dynamics1' */
  INDI_allocator_X.actuator_dynamics1_CSTATE[0] = INDI_allocator_P.w0[0];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_actuator' */
  INDI_allocator_X.lowpass_filter_actuator_CSTATE[0] =
    INDI_allocator_P.lowpass_filter_actuator_Initial;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_indi' */
  INDI_allocator_X.lowpass_filter_indi_CSTATE[0] =
    INDI_allocator_P.lowpass_filter_indi_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/actuator_dynamics1' */
  INDI_allocator_X.actuator_dynamics1_CSTATE[1] = INDI_allocator_P.w0[1];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_actuator' */
  INDI_allocator_X.lowpass_filter_actuator_CSTATE[1] =
    INDI_allocator_P.lowpass_filter_actuator_Initial;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_indi' */
  INDI_allocator_X.lowpass_filter_indi_CSTATE[1] =
    INDI_allocator_P.lowpass_filter_indi_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/actuator_dynamics1' */
  INDI_allocator_X.actuator_dynamics1_CSTATE[2] = INDI_allocator_P.w0[2];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_actuator' */
  INDI_allocator_X.lowpass_filter_actuator_CSTATE[2] =
    INDI_allocator_P.lowpass_filter_actuator_Initial;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_indi' */
  INDI_allocator_X.lowpass_filter_indi_CSTATE[2] =
    INDI_allocator_P.lowpass_filter_indi_InitialCond;

  /* InitializeConditions for StateSpace: '<S1>/actuator_dynamics1' */
  INDI_allocator_X.actuator_dynamics1_CSTATE[3] = INDI_allocator_P.w0[3];

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_actuator' */
  INDI_allocator_X.lowpass_filter_actuator_CSTATE[3] =
    INDI_allocator_P.lowpass_filter_actuator_Initial;

  /* InitializeConditions for StateSpace: '<S1>/lowpass_filter_indi' */
  INDI_allocator_X.lowpass_filter_indi_CSTATE[3] =
    INDI_allocator_P.lowpass_filter_indi_InitialCond;

  /* End of SystemInitialize for SubSystem: '<Root>/INDI_allocator' */
}

/* Model terminate function */
void INDI_allocatorModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
INDI_allocatorModelClass::INDI_allocatorModelClass()
{
  static const P_INDI_allocator_T INDI_allocator_P_temp = {
    /* Variable: G_init
     * Referenced by: '<S1>/Memory2'
     */
    { 15.0, -15.0, -15.0, 15.0 },

    /* Variable: G_init_lms
     * Referenced by: '<S1>/Memory3'
     */
    { 15.0, 14.0, -0.25, 1.0, -15.0, 14.0, 0.25, 1.0, -15.0, -14.0, -0.25, 1.0,
      15.0, -14.0, 0.25, 1.0 },

    /* Variable: P_G_init
     * Referenced by: '<S1>/Memory1'
     */
    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },

    /* Variable: para
     * Referenced by: '<S1>/MATLAB Function'
     */
    { 0.374, 0.0875, 0.115, 0.075, -1.0, 9.8124 },

    /* Variable: t_G_est
     * Referenced by:
     *   '<S1>/lowpass_filter_est '
     *   '<S1>/lowpass_filter_est 1'
     */
    0.1,

    /* Variable: w0
     * Referenced by:
     *   '<S1>/actuator_dynamics1'
     *   '<S1>/lowpass_filter1'
     */
    { 695.0, 695.0, 695.0, 695.0 },

    /* Expression: eye(3)
     * Referenced by: '<S1>/lowpass_filter '
     */
    { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

    /* Expression: 0
     * Referenced by: '<S1>/lowpass_filter '
     */
    0.0,

    /* Expression: eye(4)
     * Referenced by: '<S1>/lowpass_filter_est '
     */
    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },

    /* Expression: 0
     * Referenced by: '<S1>/lowpass_filter_est '
     */
    0.0,

    /* Expression: eye(4)
     * Referenced by: '<S1>/lowpass_filter_est 1'
     */
    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },

    /* Expression: 0
     * Referenced by: '<S1>/lowpass_filter_est 1'
     */
    0.0,

    /* Computed Parameter: lowpass_filter1_A
     * Referenced by: '<S1>/lowpass_filter1'
     */
    { -1.25, -1.25, -1.25, -1.25 },

    /* Computed Parameter: lowpass_filter1_B
     * Referenced by: '<S1>/lowpass_filter1'
     */
    { 1.25, 1.25, 1.25, 1.25 },

    /* Computed Parameter: lowpass_filter1_C
     * Referenced by: '<S1>/lowpass_filter1'
     */
    { 1.0, 1.0, 1.0, 1.0 },

    /* Expression: [0 0 0 0]'
     * Referenced by: '<S1>/Memory'
     */
    { 0.0, 0.0, 0.0, 0.0 },

    /* Expression: eye(4)
     * Referenced by: '<S1>/actuator_dynamics1'
     */
    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },

    /* Expression: eye(4)
     * Referenced by: '<S1>/lowpass_filter_actuator'
     */
    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },

    /* Expression: 0
     * Referenced by: '<S1>/lowpass_filter_actuator'
     */
    0.0,

    /* Expression: eye(4)
     * Referenced by: '<S1>/lowpass_filter_indi'
     */
    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },

    /* Expression: 0
     * Referenced by: '<S1>/lowpass_filter_indi'
     */
    0.0
  };                                   /* Modifiable parameters */

  /* Initialize tunable parameters */
  INDI_allocator_P = INDI_allocator_P_temp;
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
