/*
 * INDI_allocator.h
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

#ifndef RTW_HEADER_INDI_allocator_h_
#define RTW_HEADER_INDI_allocator_h_
#include <cmath>
#include <string.h>
#ifndef INDI_allocator_COMMON_INCLUDES_
# define INDI_allocator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* INDI_allocator_COMMON_INCLUDES_ */

#include "INDI_allocator_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
# define rtmGetOdeY(rtm)               ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
# define rtmSetOdeY(rtm, val)          ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T lowpass_filter[3];            /* '<S1>/lowpass_filter ' */
  real_T Derivative[3];                /* '<S1>/Derivative' */
  real_T lowpass_filter_est[4];        /* '<S1>/lowpass_filter_est ' */
  real_T Memory3[16];                  /* '<S1>/Memory3' */
  real_T lowpass_filter1[4];           /* '<S1>/lowpass_filter1' */
  real_T SumofElements;                /* '<S1>/Sum of Elements' */
  real_T Memory[4];                    /* '<S1>/Memory' */
  real_T Memory2[4];                   /* '<S1>/Memory2' */
  real_T Memory1[16];                  /* '<S1>/Memory1' */
  real_T TmpSignalConversionAtlowpass_fi[4];
  real_T w[4];                         /* '<S1>/actuator_dynamics1' */
  real_T lowpass_filter_actuator[4];   /* '<S1>/lowpass_filter_actuator' */
  real_T lowpass_filter_indi[4];       /* '<S1>/lowpass_filter_indi' */
  real_T G1[16];                       /* '<S1>/MATLAB Function2' */
  real_T G_out[4];                     /* '<S1>/MATLAB Function1' */
  real_T P_out[16];                    /* '<S1>/MATLAB Function1' */
  real_T w_cmd[4];                     /* '<S1>/MATLAB Function' */
  real_T du[4];                        /* '<S1>/MATLAB Function' */
} B_INDI_allocator_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T TimeStampA;                   /* '<S1>/Derivative' */
  real_T LastUAtTimeA[3];              /* '<S1>/Derivative' */
  real_T TimeStampB;                   /* '<S1>/Derivative' */
  real_T LastUAtTimeB[3];              /* '<S1>/Derivative' */
  real_T TimeStampA_n;                 /* '<S1>/Derivative2' */
  real_T LastUAtTimeA_c[4];            /* '<S1>/Derivative2' */
  real_T TimeStampB_m;                 /* '<S1>/Derivative2' */
  real_T LastUAtTimeB_p[4];            /* '<S1>/Derivative2' */
  real_T Memory3_PreviousInput[16];    /* '<S1>/Memory3' */
  real_T Memory_PreviousInput[4];      /* '<S1>/Memory' */
  real_T Memory2_PreviousInput[4];     /* '<S1>/Memory2' */
  real_T Memory1_PreviousInput[16];    /* '<S1>/Memory1' */
} DW_INDI_allocator_T;

/* Continuous states (default storage) */
typedef struct {
  real_T lowpass_filter_CSTATE[3];     /* '<S1>/lowpass_filter ' */
  real_T lowpass_filter_est_CSTATE[4]; /* '<S1>/lowpass_filter_est ' */
  real_T lowpass_filter_est1_CSTATE[4];/* '<S1>/lowpass_filter_est 1' */
  real_T lowpass_filter1_CSTATE[4];    /* '<S1>/lowpass_filter1' */
  real_T actuator_dynamics1_CSTATE[4]; /* '<S1>/actuator_dynamics1' */
  real_T lowpass_filter_actuator_CSTATE[4];/* '<S1>/lowpass_filter_actuator' */
  real_T lowpass_filter_indi_CSTATE[4];/* '<S1>/lowpass_filter_indi' */
} X_INDI_allocator_T;

/* State derivatives (default storage) */
typedef struct {
  real_T lowpass_filter_CSTATE[3];     /* '<S1>/lowpass_filter ' */
  real_T lowpass_filter_est_CSTATE[4]; /* '<S1>/lowpass_filter_est ' */
  real_T lowpass_filter_est1_CSTATE[4];/* '<S1>/lowpass_filter_est 1' */
  real_T lowpass_filter1_CSTATE[4];    /* '<S1>/lowpass_filter1' */
  real_T actuator_dynamics1_CSTATE[4]; /* '<S1>/actuator_dynamics1' */
  real_T lowpass_filter_actuator_CSTATE[4];/* '<S1>/lowpass_filter_actuator' */
  real_T lowpass_filter_indi_CSTATE[4];/* '<S1>/lowpass_filter_indi' */
} XDot_INDI_allocator_T;

/* State disabled  */
typedef struct {
  boolean_T lowpass_filter_CSTATE[3];  /* '<S1>/lowpass_filter ' */
  boolean_T lowpass_filter_est_CSTATE[4];/* '<S1>/lowpass_filter_est ' */
  boolean_T lowpass_filter_est1_CSTATE[4];/* '<S1>/lowpass_filter_est 1' */
  boolean_T lowpass_filter1_CSTATE[4]; /* '<S1>/lowpass_filter1' */
  boolean_T actuator_dynamics1_CSTATE[4];/* '<S1>/actuator_dynamics1' */
  boolean_T lowpass_filter_actuator_CSTATE[4];/* '<S1>/lowpass_filter_actuator' */
  boolean_T lowpass_filter_indi_CSTATE[4];/* '<S1>/lowpass_filter_indi' */
} XDis_INDI_allocator_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S1>/actuator_dynamics1'
   *   '<S1>/lowpass_filter_actuator'
   *   '<S1>/lowpass_filter_est '
   *   '<S1>/lowpass_filter_est 1'
   *   '<S1>/lowpass_filter_indi'
   */
  real_T pooled1[16];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by: '<S1>/lowpass_filter '
   */
  real_T pooled2[9];
} ConstP_INDI_allocator_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T pqr[3];                       /* '<Root>/pqr' */
  real_T u_pqr[3];                     /* '<Root>/pqr_dot_cmd' */
  real_T thrust_cmd;                   /* '<Root>/thrust_cmd' */
  real_T Az;                           /* '<Root>/Az' */
} ExtU_INDI_allocator_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T w_cmd[4];                     /* '<Root>/w_cmd' */
  real_T w_cmd_px4[4];                 /* '<Root>/w_cmd_px4' */
} ExtY_INDI_allocator_T;

/* Parameters (default storage) */
struct P_INDI_allocator_T_ {
  real_T G_init[4];                    /* Variable: G_init
                                        * Referenced by: '<S1>/Memory2'
                                        */
  real_T G_init_lms[16];               /* Variable: G_init_lms
                                        * Referenced by: '<S1>/Memory3'
                                        */
  real_T P_G_init[16];                 /* Variable: P_G_init
                                        * Referenced by: '<S1>/Memory1'
                                        */
  real_T para[6];                      /* Variable: para
                                        * Referenced by: '<S1>/MATLAB Function'
                                        */
  real_T t_G_est;                      /* Variable: t_G_est
                                        * Referenced by:
                                        *   '<S1>/lowpass_filter_est '
                                        *   '<S1>/lowpass_filter_est 1'
                                        */
  real_T w0[4];                        /* Variable: w0
                                        * Referenced by:
                                        *   '<S1>/actuator_dynamics1'
                                        *   '<S1>/lowpass_filter1'
                                        */
  real_T lowpass_filter_C[9];          /* Expression: eye(3)
                                        * Referenced by: '<S1>/lowpass_filter '
                                        */
  real_T lowpass_filter_InitialCondition;/* Expression: 0
                                          * Referenced by: '<S1>/lowpass_filter '
                                          */
  real_T lowpass_filter_est_C[16];     /* Expression: eye(4)
                                        * Referenced by: '<S1>/lowpass_filter_est '
                                        */
  real_T lowpass_filter_est_InitialCondi;/* Expression: 0
                                          * Referenced by: '<S1>/lowpass_filter_est '
                                          */
  real_T lowpass_filter_est1_C[16];    /* Expression: eye(4)
                                        * Referenced by: '<S1>/lowpass_filter_est 1'
                                        */
  real_T lowpass_filter_est1_InitialCond;/* Expression: 0
                                          * Referenced by: '<S1>/lowpass_filter_est 1'
                                          */
  real_T lowpass_filter1_A[4];         /* Computed Parameter: lowpass_filter1_A
                                        * Referenced by: '<S1>/lowpass_filter1'
                                        */
  real_T lowpass_filter1_B[4];         /* Computed Parameter: lowpass_filter1_B
                                        * Referenced by: '<S1>/lowpass_filter1'
                                        */
  real_T lowpass_filter1_C[4];         /* Computed Parameter: lowpass_filter1_C
                                        * Referenced by: '<S1>/lowpass_filter1'
                                        */
  real_T Memory_InitialCondition[4];   /* Expression: [0 0 0 0]'
                                        * Referenced by: '<S1>/Memory'
                                        */
  real_T actuator_dynamics1_C[16];     /* Expression: eye(4)
                                        * Referenced by: '<S1>/actuator_dynamics1'
                                        */
  real_T lowpass_filter_actuator_C[16];/* Expression: eye(4)
                                        * Referenced by: '<S1>/lowpass_filter_actuator'
                                        */
  real_T lowpass_filter_actuator_Initial;/* Expression: 0
                                          * Referenced by: '<S1>/lowpass_filter_actuator'
                                          */
  real_T lowpass_filter_indi_C[16];    /* Expression: eye(4)
                                        * Referenced by: '<S1>/lowpass_filter_indi'
                                        */
  real_T lowpass_filter_indi_InitialCond;/* Expression: 0
                                          * Referenced by: '<S1>/lowpass_filter_indi'
                                          */
};

/* Real-time Model Data Structure */
struct tag_RTM_INDI_allocator_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_INDI_allocator_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[27];
  real_T odeF[3][27];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    struct {
      uint8_T TID[3];
    } TaskCounters;

    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[3];
  } Timing;
};

/* Constant parameters (default storage) */
extern const ConstP_INDI_allocator_T INDI_allocator_ConstP;

/* Class declaration for model INDI_allocator */
class INDI_allocatorModelClass {
  /* public data and function members */
 public:
  /* External inputs */
  ExtU_INDI_allocator_T INDI_allocator_U;

  /* External outputs */
  ExtY_INDI_allocator_T INDI_allocator_Y;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  INDI_allocatorModelClass();

  /* Destructor */
  ~INDI_allocatorModelClass();

  /* Real-Time Model get method */
  RT_MODEL_INDI_allocator_T * getRTM();

  /* private data and function members */
 private:
  /* Tunable parameters */
  P_INDI_allocator_T INDI_allocator_P;

  /* Block signals */
  B_INDI_allocator_T INDI_allocator_B;

  /* Block states */
  DW_INDI_allocator_T INDI_allocator_DW;
  X_INDI_allocator_T INDI_allocator_X; /* Block continuous states */

  /* Real-Time Model */
  RT_MODEL_INDI_allocator_T INDI_allocator_M;

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void INDI_allocator_derivatives();
};

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator')    - opens subsystem quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator
 * hilite_system('quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller'
 * '<S1>'   : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator'
 * '<S2>'   : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator/FD'
 * '<S3>'   : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator/MATLAB Function'
 * '<S4>'   : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator/MATLAB Function1'
 * '<S5>'   : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator/MATLAB Function2'
 * '<S6>'   : 'quadrotor/Controller/Primary_axis_INDI_controller/attitude_controller/INDI_allocator/fail_id'
 */
#endif                                 /* RTW_HEADER_INDI_allocator_h_ */
