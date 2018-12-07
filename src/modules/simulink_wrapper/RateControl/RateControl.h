/*
 * RateControl.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "RateControl".
 *
 * Model version              : 1.491
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Fri Nov 30 17:32:08 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Generic->Custom
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_RateControl_h_
#define RTW_HEADER_RateControl_h_
#include <string.h>
#include <cmath>
#include <float.h>
#include <math.h>
#include <stddef.h>
#ifndef RateControl_COMMON_INCLUDES_
# define RateControl_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* RateControl_COMMON_INCLUDES_ */

#include "RateControl_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmCounterLimit
# define rtmCounterLimit(rtm, idx)     ((rtm)->Timing.TaskCounters.cLimit[(idx)])
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmStepTask
# define rtmStepTask(rtm, idx)         ((rtm)->Timing.TaskCounters.TID[(idx)] == 0)
#endif

#ifndef rtmTaskCounter
# define rtmTaskCounter(rtm, idx)      ((rtm)->Timing.TaskCounters.TID[(idx)])
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Diff[3];                      /* '<S2>/Diff' */
  real_T est_Omega_ddot_pred[4];       /* '<Root>/MATLAB Function1' */
  real_T MathFunction;                 /* '<S18>/Math Function' */
  real_T OverwriteValues[3];           /* '<S19>/Overwrite Values' */
} B_RateControl_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T H_est_actuators_DSTATE[8];    /* '<Root>/H_est_actuators' */
  real_T UD_DSTATE[4];                 /* '<S3>/UD' */
  real_T UD_DSTATE_k[4];               /* '<S5>/UD' */
  real_T H_est_rates_DSTATE[6];        /* '<Root>/H_est_rates' */
  real_T UD_DSTATE_b[3];               /* '<S4>/UD' */
  real_T UD_DSTATE_n[3];               /* '<S2>/UD' */
  real_T H_est_accel_DSTATE[2];        /* '<Root>/H_est_accel' */
  real_T UD_DSTATE_d;                  /* '<S6>/UD' */
  real_T Buffer1_CircBuf[400];         /* '<S8>/Buffer1' */
  real_T H_rates_DSTATE[6];            /* '<Root>/H_rates' */
  real_T UD_DSTATE_m[3];               /* '<S1>/UD' */
  real_T H_accel_DSTATE[2];            /* '<Root>/H_accel' */
  real_T Integrator_DSTATE;            /* '<S170>/Integrator' */
  real_T UD_DSTATE_b2;                 /* '<S149>/UD' */
  real_T Integrator_DSTATE_f;          /* '<S74>/Integrator' */
  real_T UD_DSTATE_l;                  /* '<S53>/UD' */
  real_T Integrator_DSTATE_b;          /* '<S266>/Integrator' */
  real_T UD_DSTATE_bg;                 /* '<S245>/UD' */
  real_T H_actuators_DSTATE[8];        /* '<Root>/H_actuators' */
  real_T Buffer2_CircBuf[400];         /* '<S8>/Buffer2' */
  real_T DelayLine_Buff[3];            /* '<S19>/Delay Line' */
  real_T Delay_DSTATE[8];              /* '<Root>/Delay' */
  real_T actuatordynamics_DSTATE[4];   /* '<Root>/actuator dynamics' */
  real_T Maximum_Valdata;              /* '<S18>/Maximum' */
  real_T Memory3_PreviousInput[32];    /* '<Root>/Memory3' */
  real_T du_last[4];                   /* '<Root>/INDI_allocator' */
  real_T Correlation_DWORK1[200];      /* '<S18>/Correlation' */
  int32_T Buffer1_inBufPtrIdx;         /* '<S8>/Buffer1' */
  int32_T Buffer1_outBufPtrIdx;        /* '<S8>/Buffer1' */
  int32_T Buffer1_bufferLength;        /* '<S8>/Buffer1' */
  int32_T Buffer2_inBufPtrIdx;         /* '<S8>/Buffer2' */
  int32_T Buffer2_outBufPtrIdx;        /* '<S8>/Buffer2' */
  int32_T Buffer2_bufferLength;        /* '<S8>/Buffer2' */
  int32_T DelayLine_BUFF_OFFSET;       /* '<S19>/Delay Line' */
} DW_RateControl_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by: '<Root>/actuator dynamics'
   */
  real_T pooled1[16];

  /* Expression: control_eff_matrix(rateControlParams)
   * Referenced by: '<Root>/Memory3'
   */
  real_T Memory3_InitialCondition[32];

  /* Expression: eye(4)
   * Referenced by: '<Root>/actuator dynamics'
   */
  real_T actuatordynamics_C[16];
} ConstP_RateControl_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T rates[3];                   /* '<Root>/rates' */
  real32_T rates_sp[3];                /* '<Root>/rates_sp' */
  real32_T accel_z;                    /* '<Root>/accel_z' */
  real32_T thrust_sp;                  /* '<Root>/thrust_sp' */
  real_T wRotor[4];                    /* '<Root>/wRotor' */
} ExtU_RateControl_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T w_cmd[4];                     /* '<Root>/w_cmd' */
  real32_T actuators_control[4];       /* '<Root>/actuators_control' */
  real_T G[32];                        /* '<Root>/G' */
} ExtY_RateControl_T;

/* Real-time Model Data Structure */
struct tag_RTM_RateControl_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint16_T TID[2];
      uint16_T cLimit[2];
    } TaskCounters;
  } Timing;
};

/* Constant parameters (default storage) */
extern const ConstP_RateControl_T RateControl_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern RateControlParamsType RateControlParams;/* Variable: RateControlParams
                                                * Referenced by:
                                                *   '<Root>/controlEffMatrix'
                                                *   '<Root>/actuator dynamics'
                                                *   '<S91>/Proportional Gain'
                                                *   '<S187>/Proportional Gain'
                                                *   '<S283>/Proportional Gain'
                                                */

/* Class declaration for model RateControl */
class RateControlModelClass {
  /* public data and function members */
 public:
  /* External inputs */
  ExtU_RateControl_T RateControl_U;

  /* External outputs */
  ExtY_RateControl_T RateControl_Y;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step0();

  /* model step function */
  void step1();

  /* model terminate function */
  void terminate();

  /* Constructor */
  RateControlModelClass();

  /* Destructor */
  ~RateControlModelClass();

  /* Real-Time Model get method */
  RT_MODEL_RateControl_T * getRTM();

  /* private data and function members */
 private:
  /* Block signals */
  B_RateControl_T RateControl_B;

  /* Block states */
  DW_RateControl_T RateControl_DW;

  /* Real-Time Model */
  RT_MODEL_RateControl_T RateControl_M;

  /* private member function(s) for subsystem '<Root>'*/
  real_T RateControl_xnrm2(int32_T n, const real_T x[16], int32_T ix0);
  real_T RateControl_xnrm2_n(int32_T n, const real_T x[4], int32_T ix0);
  void RateControl_xaxpy_fy(int32_T n, real_T a, const real_T x[4], int32_T ix0,
    real_T y[16], int32_T iy0);
  void RateControl_xaxpy_f(int32_T n, real_T a, const real_T x[16], int32_T ix0,
    real_T y[4], int32_T iy0);
  real_T RateControl_xdotc(int32_T n, const real_T x[16], int32_T ix0, const
    real_T y[16], int32_T iy0);
  void RateControl_xaxpy(int32_T n, real_T a, int32_T ix0, real_T y[16], int32_T
    iy0);
  void RateControl_xscal(real_T a, real_T x[16], int32_T ix0);
  void RateControl_xswap(real_T x[16], int32_T ix0, int32_T iy0);
  void RateControl_xrotg(real_T *a, real_T *b, real_T *c, real_T *s);
  void RateControl_xrot(real_T x[16], int32_T ix0, int32_T iy0, real_T c, real_T
                        s);
  void RateControl_svd(const real_T A[16], real_T U[16], real_T s[4], real_T V
                       [16]);
  void RateControl_pinv(const real_T A[16], real_T X[16]);
};

// Model step wrapper function for compatibility with a static main program
void RateControl_step(RateControlModelClass & RateControl_Obj, int_T tid);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'RateControl'
 * '<S1>'   : 'RateControl/Discrete Derivative'
 * '<S2>'   : 'RateControl/Discrete Derivative1'
 * '<S3>'   : 'RateControl/Discrete Derivative2'
 * '<S4>'   : 'RateControl/Discrete Derivative4'
 * '<S5>'   : 'RateControl/Discrete Derivative5'
 * '<S6>'   : 'RateControl/Discrete Derivative6'
 * '<S7>'   : 'RateControl/FD'
 * '<S8>'   : 'RateControl/Find Delay'
 * '<S9>'   : 'RateControl/INDI_allocator'
 * '<S10>'  : 'RateControl/MATLAB Function'
 * '<S11>'  : 'RateControl/MATLAB Function1'
 * '<S12>'  : 'RateControl/PD_pitch'
 * '<S13>'  : 'RateControl/PD_roll'
 * '<S14>'  : 'RateControl/PD_yaw'
 * '<S15>'  : 'RateControl/controlEffMatrix'
 * '<S16>'  : 'RateControl/lms'
 * '<S17>'  : 'RateControl/wobbler'
 * '<S18>'  : 'RateControl/Find Delay/Compute Delay'
 * '<S19>'  : 'RateControl/Find Delay/Enable Logic'
 * '<S20>'  : 'RateControl/PD_pitch/Anti-windup'
 * '<S21>'  : 'RateControl/PD_pitch/D Gain'
 * '<S22>'  : 'RateControl/PD_pitch/Filter'
 * '<S23>'  : 'RateControl/PD_pitch/Filter ICs'
 * '<S24>'  : 'RateControl/PD_pitch/I Gain'
 * '<S25>'  : 'RateControl/PD_pitch/Ideal P Gain'
 * '<S26>'  : 'RateControl/PD_pitch/Ideal P Gain Fdbk'
 * '<S27>'  : 'RateControl/PD_pitch/Integrator'
 * '<S28>'  : 'RateControl/PD_pitch/Integrator ICs'
 * '<S29>'  : 'RateControl/PD_pitch/N Copy'
 * '<S30>'  : 'RateControl/PD_pitch/N Gain'
 * '<S31>'  : 'RateControl/PD_pitch/P Copy'
 * '<S32>'  : 'RateControl/PD_pitch/Parallel P Gain'
 * '<S33>'  : 'RateControl/PD_pitch/Reset Signal'
 * '<S34>'  : 'RateControl/PD_pitch/Saturation'
 * '<S35>'  : 'RateControl/PD_pitch/Saturation Fdbk'
 * '<S36>'  : 'RateControl/PD_pitch/Sum'
 * '<S37>'  : 'RateControl/PD_pitch/Sum Fdbk'
 * '<S38>'  : 'RateControl/PD_pitch/Tracking Mode'
 * '<S39>'  : 'RateControl/PD_pitch/Tracking Mode Sum'
 * '<S40>'  : 'RateControl/PD_pitch/postSat Signal'
 * '<S41>'  : 'RateControl/PD_pitch/preSat Signal'
 * '<S42>'  : 'RateControl/PD_pitch/Anti-windup/Back Calculation'
 * '<S43>'  : 'RateControl/PD_pitch/Anti-windup/Cont. Clamping Ideal'
 * '<S44>'  : 'RateControl/PD_pitch/Anti-windup/Cont. Clamping Parallel'
 * '<S45>'  : 'RateControl/PD_pitch/Anti-windup/Disabled'
 * '<S46>'  : 'RateControl/PD_pitch/Anti-windup/Disc. Clamping Ideal'
 * '<S47>'  : 'RateControl/PD_pitch/Anti-windup/Disc. Clamping Parallel'
 * '<S48>'  : 'RateControl/PD_pitch/Anti-windup/Passthrough'
 * '<S49>'  : 'RateControl/PD_pitch/D Gain/Disabled'
 * '<S50>'  : 'RateControl/PD_pitch/D Gain/External Parameters'
 * '<S51>'  : 'RateControl/PD_pitch/D Gain/Internal Parameters'
 * '<S52>'  : 'RateControl/PD_pitch/Filter/Cont. Filter'
 * '<S53>'  : 'RateControl/PD_pitch/Filter/Differentiator'
 * '<S54>'  : 'RateControl/PD_pitch/Filter/Disabled'
 * '<S55>'  : 'RateControl/PD_pitch/Filter/Disc. Backward Euler Filter'
 * '<S56>'  : 'RateControl/PD_pitch/Filter/Disc. Forward Euler Filter'
 * '<S57>'  : 'RateControl/PD_pitch/Filter/Disc. Trapezoidal Filter'
 * '<S58>'  : 'RateControl/PD_pitch/Filter ICs/Disabled'
 * '<S59>'  : 'RateControl/PD_pitch/Filter ICs/External IC'
 * '<S60>'  : 'RateControl/PD_pitch/Filter ICs/Internal IC - Differentiator'
 * '<S61>'  : 'RateControl/PD_pitch/Filter ICs/Internal IC - Filter'
 * '<S62>'  : 'RateControl/PD_pitch/I Gain/Disabled'
 * '<S63>'  : 'RateControl/PD_pitch/I Gain/External Parameters'
 * '<S64>'  : 'RateControl/PD_pitch/I Gain/Internal Parameters'
 * '<S65>'  : 'RateControl/PD_pitch/Ideal P Gain/External Parameters'
 * '<S66>'  : 'RateControl/PD_pitch/Ideal P Gain/Internal Parameters'
 * '<S67>'  : 'RateControl/PD_pitch/Ideal P Gain/Passthrough'
 * '<S68>'  : 'RateControl/PD_pitch/Ideal P Gain Fdbk/Disabled'
 * '<S69>'  : 'RateControl/PD_pitch/Ideal P Gain Fdbk/External Parameters'
 * '<S70>'  : 'RateControl/PD_pitch/Ideal P Gain Fdbk/Internal Parameters'
 * '<S71>'  : 'RateControl/PD_pitch/Ideal P Gain Fdbk/Passthrough'
 * '<S72>'  : 'RateControl/PD_pitch/Integrator/Continuous'
 * '<S73>'  : 'RateControl/PD_pitch/Integrator/Disabled'
 * '<S74>'  : 'RateControl/PD_pitch/Integrator/Discrete'
 * '<S75>'  : 'RateControl/PD_pitch/Integrator ICs/Disabled'
 * '<S76>'  : 'RateControl/PD_pitch/Integrator ICs/External IC'
 * '<S77>'  : 'RateControl/PD_pitch/Integrator ICs/Internal IC'
 * '<S78>'  : 'RateControl/PD_pitch/N Copy/Disabled'
 * '<S79>'  : 'RateControl/PD_pitch/N Copy/Disabled wSignal Specification'
 * '<S80>'  : 'RateControl/PD_pitch/N Copy/External Parameters'
 * '<S81>'  : 'RateControl/PD_pitch/N Copy/Internal Parameters'
 * '<S82>'  : 'RateControl/PD_pitch/N Gain/Disabled'
 * '<S83>'  : 'RateControl/PD_pitch/N Gain/External Parameters'
 * '<S84>'  : 'RateControl/PD_pitch/N Gain/Internal Parameters'
 * '<S85>'  : 'RateControl/PD_pitch/N Gain/Passthrough'
 * '<S86>'  : 'RateControl/PD_pitch/P Copy/Disabled'
 * '<S87>'  : 'RateControl/PD_pitch/P Copy/External Parameters Ideal'
 * '<S88>'  : 'RateControl/PD_pitch/P Copy/Internal Parameters Ideal'
 * '<S89>'  : 'RateControl/PD_pitch/Parallel P Gain/Disabled'
 * '<S90>'  : 'RateControl/PD_pitch/Parallel P Gain/External Parameters'
 * '<S91>'  : 'RateControl/PD_pitch/Parallel P Gain/Internal Parameters'
 * '<S92>'  : 'RateControl/PD_pitch/Parallel P Gain/Passthrough'
 * '<S93>'  : 'RateControl/PD_pitch/Reset Signal/Disabled'
 * '<S94>'  : 'RateControl/PD_pitch/Reset Signal/External Reset'
 * '<S95>'  : 'RateControl/PD_pitch/Saturation/Enabled'
 * '<S96>'  : 'RateControl/PD_pitch/Saturation/Passthrough'
 * '<S97>'  : 'RateControl/PD_pitch/Saturation Fdbk/Disabled'
 * '<S98>'  : 'RateControl/PD_pitch/Saturation Fdbk/Enabled'
 * '<S99>'  : 'RateControl/PD_pitch/Saturation Fdbk/Passthrough'
 * '<S100>' : 'RateControl/PD_pitch/Sum/Passthrough_I'
 * '<S101>' : 'RateControl/PD_pitch/Sum/Passthrough_P'
 * '<S102>' : 'RateControl/PD_pitch/Sum/Sum_PD'
 * '<S103>' : 'RateControl/PD_pitch/Sum/Sum_PI'
 * '<S104>' : 'RateControl/PD_pitch/Sum/Sum_PID'
 * '<S105>' : 'RateControl/PD_pitch/Sum Fdbk/Disabled'
 * '<S106>' : 'RateControl/PD_pitch/Sum Fdbk/Enabled'
 * '<S107>' : 'RateControl/PD_pitch/Sum Fdbk/Passthrough'
 * '<S108>' : 'RateControl/PD_pitch/Tracking Mode/Disabled'
 * '<S109>' : 'RateControl/PD_pitch/Tracking Mode/Enabled'
 * '<S110>' : 'RateControl/PD_pitch/Tracking Mode Sum/Passthrough'
 * '<S111>' : 'RateControl/PD_pitch/Tracking Mode Sum/Tracking Mode'
 * '<S112>' : 'RateControl/PD_pitch/postSat Signal/Feedback_Path'
 * '<S113>' : 'RateControl/PD_pitch/postSat Signal/Forward_Path'
 * '<S114>' : 'RateControl/PD_pitch/preSat Signal/Feedback_Path'
 * '<S115>' : 'RateControl/PD_pitch/preSat Signal/Forward_Path'
 * '<S116>' : 'RateControl/PD_roll/Anti-windup'
 * '<S117>' : 'RateControl/PD_roll/D Gain'
 * '<S118>' : 'RateControl/PD_roll/Filter'
 * '<S119>' : 'RateControl/PD_roll/Filter ICs'
 * '<S120>' : 'RateControl/PD_roll/I Gain'
 * '<S121>' : 'RateControl/PD_roll/Ideal P Gain'
 * '<S122>' : 'RateControl/PD_roll/Ideal P Gain Fdbk'
 * '<S123>' : 'RateControl/PD_roll/Integrator'
 * '<S124>' : 'RateControl/PD_roll/Integrator ICs'
 * '<S125>' : 'RateControl/PD_roll/N Copy'
 * '<S126>' : 'RateControl/PD_roll/N Gain'
 * '<S127>' : 'RateControl/PD_roll/P Copy'
 * '<S128>' : 'RateControl/PD_roll/Parallel P Gain'
 * '<S129>' : 'RateControl/PD_roll/Reset Signal'
 * '<S130>' : 'RateControl/PD_roll/Saturation'
 * '<S131>' : 'RateControl/PD_roll/Saturation Fdbk'
 * '<S132>' : 'RateControl/PD_roll/Sum'
 * '<S133>' : 'RateControl/PD_roll/Sum Fdbk'
 * '<S134>' : 'RateControl/PD_roll/Tracking Mode'
 * '<S135>' : 'RateControl/PD_roll/Tracking Mode Sum'
 * '<S136>' : 'RateControl/PD_roll/postSat Signal'
 * '<S137>' : 'RateControl/PD_roll/preSat Signal'
 * '<S138>' : 'RateControl/PD_roll/Anti-windup/Back Calculation'
 * '<S139>' : 'RateControl/PD_roll/Anti-windup/Cont. Clamping Ideal'
 * '<S140>' : 'RateControl/PD_roll/Anti-windup/Cont. Clamping Parallel'
 * '<S141>' : 'RateControl/PD_roll/Anti-windup/Disabled'
 * '<S142>' : 'RateControl/PD_roll/Anti-windup/Disc. Clamping Ideal'
 * '<S143>' : 'RateControl/PD_roll/Anti-windup/Disc. Clamping Parallel'
 * '<S144>' : 'RateControl/PD_roll/Anti-windup/Passthrough'
 * '<S145>' : 'RateControl/PD_roll/D Gain/Disabled'
 * '<S146>' : 'RateControl/PD_roll/D Gain/External Parameters'
 * '<S147>' : 'RateControl/PD_roll/D Gain/Internal Parameters'
 * '<S148>' : 'RateControl/PD_roll/Filter/Cont. Filter'
 * '<S149>' : 'RateControl/PD_roll/Filter/Differentiator'
 * '<S150>' : 'RateControl/PD_roll/Filter/Disabled'
 * '<S151>' : 'RateControl/PD_roll/Filter/Disc. Backward Euler Filter'
 * '<S152>' : 'RateControl/PD_roll/Filter/Disc. Forward Euler Filter'
 * '<S153>' : 'RateControl/PD_roll/Filter/Disc. Trapezoidal Filter'
 * '<S154>' : 'RateControl/PD_roll/Filter ICs/Disabled'
 * '<S155>' : 'RateControl/PD_roll/Filter ICs/External IC'
 * '<S156>' : 'RateControl/PD_roll/Filter ICs/Internal IC - Differentiator'
 * '<S157>' : 'RateControl/PD_roll/Filter ICs/Internal IC - Filter'
 * '<S158>' : 'RateControl/PD_roll/I Gain/Disabled'
 * '<S159>' : 'RateControl/PD_roll/I Gain/External Parameters'
 * '<S160>' : 'RateControl/PD_roll/I Gain/Internal Parameters'
 * '<S161>' : 'RateControl/PD_roll/Ideal P Gain/External Parameters'
 * '<S162>' : 'RateControl/PD_roll/Ideal P Gain/Internal Parameters'
 * '<S163>' : 'RateControl/PD_roll/Ideal P Gain/Passthrough'
 * '<S164>' : 'RateControl/PD_roll/Ideal P Gain Fdbk/Disabled'
 * '<S165>' : 'RateControl/PD_roll/Ideal P Gain Fdbk/External Parameters'
 * '<S166>' : 'RateControl/PD_roll/Ideal P Gain Fdbk/Internal Parameters'
 * '<S167>' : 'RateControl/PD_roll/Ideal P Gain Fdbk/Passthrough'
 * '<S168>' : 'RateControl/PD_roll/Integrator/Continuous'
 * '<S169>' : 'RateControl/PD_roll/Integrator/Disabled'
 * '<S170>' : 'RateControl/PD_roll/Integrator/Discrete'
 * '<S171>' : 'RateControl/PD_roll/Integrator ICs/Disabled'
 * '<S172>' : 'RateControl/PD_roll/Integrator ICs/External IC'
 * '<S173>' : 'RateControl/PD_roll/Integrator ICs/Internal IC'
 * '<S174>' : 'RateControl/PD_roll/N Copy/Disabled'
 * '<S175>' : 'RateControl/PD_roll/N Copy/Disabled wSignal Specification'
 * '<S176>' : 'RateControl/PD_roll/N Copy/External Parameters'
 * '<S177>' : 'RateControl/PD_roll/N Copy/Internal Parameters'
 * '<S178>' : 'RateControl/PD_roll/N Gain/Disabled'
 * '<S179>' : 'RateControl/PD_roll/N Gain/External Parameters'
 * '<S180>' : 'RateControl/PD_roll/N Gain/Internal Parameters'
 * '<S181>' : 'RateControl/PD_roll/N Gain/Passthrough'
 * '<S182>' : 'RateControl/PD_roll/P Copy/Disabled'
 * '<S183>' : 'RateControl/PD_roll/P Copy/External Parameters Ideal'
 * '<S184>' : 'RateControl/PD_roll/P Copy/Internal Parameters Ideal'
 * '<S185>' : 'RateControl/PD_roll/Parallel P Gain/Disabled'
 * '<S186>' : 'RateControl/PD_roll/Parallel P Gain/External Parameters'
 * '<S187>' : 'RateControl/PD_roll/Parallel P Gain/Internal Parameters'
 * '<S188>' : 'RateControl/PD_roll/Parallel P Gain/Passthrough'
 * '<S189>' : 'RateControl/PD_roll/Reset Signal/Disabled'
 * '<S190>' : 'RateControl/PD_roll/Reset Signal/External Reset'
 * '<S191>' : 'RateControl/PD_roll/Saturation/Enabled'
 * '<S192>' : 'RateControl/PD_roll/Saturation/Passthrough'
 * '<S193>' : 'RateControl/PD_roll/Saturation Fdbk/Disabled'
 * '<S194>' : 'RateControl/PD_roll/Saturation Fdbk/Enabled'
 * '<S195>' : 'RateControl/PD_roll/Saturation Fdbk/Passthrough'
 * '<S196>' : 'RateControl/PD_roll/Sum/Passthrough_I'
 * '<S197>' : 'RateControl/PD_roll/Sum/Passthrough_P'
 * '<S198>' : 'RateControl/PD_roll/Sum/Sum_PD'
 * '<S199>' : 'RateControl/PD_roll/Sum/Sum_PI'
 * '<S200>' : 'RateControl/PD_roll/Sum/Sum_PID'
 * '<S201>' : 'RateControl/PD_roll/Sum Fdbk/Disabled'
 * '<S202>' : 'RateControl/PD_roll/Sum Fdbk/Enabled'
 * '<S203>' : 'RateControl/PD_roll/Sum Fdbk/Passthrough'
 * '<S204>' : 'RateControl/PD_roll/Tracking Mode/Disabled'
 * '<S205>' : 'RateControl/PD_roll/Tracking Mode/Enabled'
 * '<S206>' : 'RateControl/PD_roll/Tracking Mode Sum/Passthrough'
 * '<S207>' : 'RateControl/PD_roll/Tracking Mode Sum/Tracking Mode'
 * '<S208>' : 'RateControl/PD_roll/postSat Signal/Feedback_Path'
 * '<S209>' : 'RateControl/PD_roll/postSat Signal/Forward_Path'
 * '<S210>' : 'RateControl/PD_roll/preSat Signal/Feedback_Path'
 * '<S211>' : 'RateControl/PD_roll/preSat Signal/Forward_Path'
 * '<S212>' : 'RateControl/PD_yaw/Anti-windup'
 * '<S213>' : 'RateControl/PD_yaw/D Gain'
 * '<S214>' : 'RateControl/PD_yaw/Filter'
 * '<S215>' : 'RateControl/PD_yaw/Filter ICs'
 * '<S216>' : 'RateControl/PD_yaw/I Gain'
 * '<S217>' : 'RateControl/PD_yaw/Ideal P Gain'
 * '<S218>' : 'RateControl/PD_yaw/Ideal P Gain Fdbk'
 * '<S219>' : 'RateControl/PD_yaw/Integrator'
 * '<S220>' : 'RateControl/PD_yaw/Integrator ICs'
 * '<S221>' : 'RateControl/PD_yaw/N Copy'
 * '<S222>' : 'RateControl/PD_yaw/N Gain'
 * '<S223>' : 'RateControl/PD_yaw/P Copy'
 * '<S224>' : 'RateControl/PD_yaw/Parallel P Gain'
 * '<S225>' : 'RateControl/PD_yaw/Reset Signal'
 * '<S226>' : 'RateControl/PD_yaw/Saturation'
 * '<S227>' : 'RateControl/PD_yaw/Saturation Fdbk'
 * '<S228>' : 'RateControl/PD_yaw/Sum'
 * '<S229>' : 'RateControl/PD_yaw/Sum Fdbk'
 * '<S230>' : 'RateControl/PD_yaw/Tracking Mode'
 * '<S231>' : 'RateControl/PD_yaw/Tracking Mode Sum'
 * '<S232>' : 'RateControl/PD_yaw/postSat Signal'
 * '<S233>' : 'RateControl/PD_yaw/preSat Signal'
 * '<S234>' : 'RateControl/PD_yaw/Anti-windup/Back Calculation'
 * '<S235>' : 'RateControl/PD_yaw/Anti-windup/Cont. Clamping Ideal'
 * '<S236>' : 'RateControl/PD_yaw/Anti-windup/Cont. Clamping Parallel'
 * '<S237>' : 'RateControl/PD_yaw/Anti-windup/Disabled'
 * '<S238>' : 'RateControl/PD_yaw/Anti-windup/Disc. Clamping Ideal'
 * '<S239>' : 'RateControl/PD_yaw/Anti-windup/Disc. Clamping Parallel'
 * '<S240>' : 'RateControl/PD_yaw/Anti-windup/Passthrough'
 * '<S241>' : 'RateControl/PD_yaw/D Gain/Disabled'
 * '<S242>' : 'RateControl/PD_yaw/D Gain/External Parameters'
 * '<S243>' : 'RateControl/PD_yaw/D Gain/Internal Parameters'
 * '<S244>' : 'RateControl/PD_yaw/Filter/Cont. Filter'
 * '<S245>' : 'RateControl/PD_yaw/Filter/Differentiator'
 * '<S246>' : 'RateControl/PD_yaw/Filter/Disabled'
 * '<S247>' : 'RateControl/PD_yaw/Filter/Disc. Backward Euler Filter'
 * '<S248>' : 'RateControl/PD_yaw/Filter/Disc. Forward Euler Filter'
 * '<S249>' : 'RateControl/PD_yaw/Filter/Disc. Trapezoidal Filter'
 * '<S250>' : 'RateControl/PD_yaw/Filter ICs/Disabled'
 * '<S251>' : 'RateControl/PD_yaw/Filter ICs/External IC'
 * '<S252>' : 'RateControl/PD_yaw/Filter ICs/Internal IC - Differentiator'
 * '<S253>' : 'RateControl/PD_yaw/Filter ICs/Internal IC - Filter'
 * '<S254>' : 'RateControl/PD_yaw/I Gain/Disabled'
 * '<S255>' : 'RateControl/PD_yaw/I Gain/External Parameters'
 * '<S256>' : 'RateControl/PD_yaw/I Gain/Internal Parameters'
 * '<S257>' : 'RateControl/PD_yaw/Ideal P Gain/External Parameters'
 * '<S258>' : 'RateControl/PD_yaw/Ideal P Gain/Internal Parameters'
 * '<S259>' : 'RateControl/PD_yaw/Ideal P Gain/Passthrough'
 * '<S260>' : 'RateControl/PD_yaw/Ideal P Gain Fdbk/Disabled'
 * '<S261>' : 'RateControl/PD_yaw/Ideal P Gain Fdbk/External Parameters'
 * '<S262>' : 'RateControl/PD_yaw/Ideal P Gain Fdbk/Internal Parameters'
 * '<S263>' : 'RateControl/PD_yaw/Ideal P Gain Fdbk/Passthrough'
 * '<S264>' : 'RateControl/PD_yaw/Integrator/Continuous'
 * '<S265>' : 'RateControl/PD_yaw/Integrator/Disabled'
 * '<S266>' : 'RateControl/PD_yaw/Integrator/Discrete'
 * '<S267>' : 'RateControl/PD_yaw/Integrator ICs/Disabled'
 * '<S268>' : 'RateControl/PD_yaw/Integrator ICs/External IC'
 * '<S269>' : 'RateControl/PD_yaw/Integrator ICs/Internal IC'
 * '<S270>' : 'RateControl/PD_yaw/N Copy/Disabled'
 * '<S271>' : 'RateControl/PD_yaw/N Copy/Disabled wSignal Specification'
 * '<S272>' : 'RateControl/PD_yaw/N Copy/External Parameters'
 * '<S273>' : 'RateControl/PD_yaw/N Copy/Internal Parameters'
 * '<S274>' : 'RateControl/PD_yaw/N Gain/Disabled'
 * '<S275>' : 'RateControl/PD_yaw/N Gain/External Parameters'
 * '<S276>' : 'RateControl/PD_yaw/N Gain/Internal Parameters'
 * '<S277>' : 'RateControl/PD_yaw/N Gain/Passthrough'
 * '<S278>' : 'RateControl/PD_yaw/P Copy/Disabled'
 * '<S279>' : 'RateControl/PD_yaw/P Copy/External Parameters Ideal'
 * '<S280>' : 'RateControl/PD_yaw/P Copy/Internal Parameters Ideal'
 * '<S281>' : 'RateControl/PD_yaw/Parallel P Gain/Disabled'
 * '<S282>' : 'RateControl/PD_yaw/Parallel P Gain/External Parameters'
 * '<S283>' : 'RateControl/PD_yaw/Parallel P Gain/Internal Parameters'
 * '<S284>' : 'RateControl/PD_yaw/Parallel P Gain/Passthrough'
 * '<S285>' : 'RateControl/PD_yaw/Reset Signal/Disabled'
 * '<S286>' : 'RateControl/PD_yaw/Reset Signal/External Reset'
 * '<S287>' : 'RateControl/PD_yaw/Saturation/Enabled'
 * '<S288>' : 'RateControl/PD_yaw/Saturation/Passthrough'
 * '<S289>' : 'RateControl/PD_yaw/Saturation Fdbk/Disabled'
 * '<S290>' : 'RateControl/PD_yaw/Saturation Fdbk/Enabled'
 * '<S291>' : 'RateControl/PD_yaw/Saturation Fdbk/Passthrough'
 * '<S292>' : 'RateControl/PD_yaw/Sum/Passthrough_I'
 * '<S293>' : 'RateControl/PD_yaw/Sum/Passthrough_P'
 * '<S294>' : 'RateControl/PD_yaw/Sum/Sum_PD'
 * '<S295>' : 'RateControl/PD_yaw/Sum/Sum_PI'
 * '<S296>' : 'RateControl/PD_yaw/Sum/Sum_PID'
 * '<S297>' : 'RateControl/PD_yaw/Sum Fdbk/Disabled'
 * '<S298>' : 'RateControl/PD_yaw/Sum Fdbk/Enabled'
 * '<S299>' : 'RateControl/PD_yaw/Sum Fdbk/Passthrough'
 * '<S300>' : 'RateControl/PD_yaw/Tracking Mode/Disabled'
 * '<S301>' : 'RateControl/PD_yaw/Tracking Mode/Enabled'
 * '<S302>' : 'RateControl/PD_yaw/Tracking Mode Sum/Passthrough'
 * '<S303>' : 'RateControl/PD_yaw/Tracking Mode Sum/Tracking Mode'
 * '<S304>' : 'RateControl/PD_yaw/postSat Signal/Feedback_Path'
 * '<S305>' : 'RateControl/PD_yaw/postSat Signal/Forward_Path'
 * '<S306>' : 'RateControl/PD_yaw/preSat Signal/Feedback_Path'
 * '<S307>' : 'RateControl/PD_yaw/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_RateControl_h_ */
