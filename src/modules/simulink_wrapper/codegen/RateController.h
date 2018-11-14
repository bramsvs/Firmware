/*
 * RateController.h
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

#ifndef RTW_HEADER_RateController_h_
#define RTW_HEADER_RateController_h_
#include <string.h>
#include <cmath>
#ifndef RateController_COMMON_INCLUDES_
# define RateController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* RateController_COMMON_INCLUDES_ */

#include "RateController_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteStateSpace_DSTATE[3]; /* '<Root>/Discrete State-Space' */
  real_T UD_DSTATE[3];                 /* '<S1>/UD' */
  real_T wRotorsensor_DSTATE[4];       /* '<Root>/wRotor sensor' */
  real_T DiscreteStateSpace2_DSTATE[4];/* '<Root>/Discrete State-Space2' */
  real_T DiscreteStateSpace3_DSTATE[4];/* '<Root>/Discrete State-Space3' */
  real_T UD_DSTATE_n[4];               /* '<S2>/UD' */
  real_T DiscreteStateSpace1_DSTATE[4];/* '<Root>/Discrete State-Space1' */
  real_T actuatordynamics_DSTATE[4];   /* '<Root>/actuator dynamics' */
  real_T actuatordynamics1_DSTATE[4];  /* '<Root>/actuator dynamics1' */
  real_T Memory_PreviousInput[4];      /* '<Root>/Memory' */
  real_T Memory3_PreviousInput[16];    /* '<Root>/Memory3' */
} DW_RateController_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: par.control_eff.init)
   * Referenced by:
   *   '<Root>/control_eff.init'
   *   '<Root>/Memory3'
   */
  real_T pooled1[16];
} ConstP_RateController_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T rates[3];                   /* '<Root>/rates' */
  real32_T rates_dot_sp[3];            /* '<Root>/rates_dot_sp' */
  real32_T thrust_sp;                  /* '<Root>/thrust_sp' */
  real32_T accel_z;                    /* '<Root>/accel_z' */
  real_T wRotor[4];                    /* '<Root>/wRotor' */
} ExtU_RateController_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T w_cmd[4];                     /* '<Root>/w_cmd' */
  real32_T actuators_control[4];       /* '<Root>/actuators_control' */
  real_T G[16];                        /* '<Root>/G' */
} ExtY_RateController_T;

/* Real-time Model Data Structure */
struct tag_RTM_RateController_T {
  const char_T *errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
  } Timing;
};

/* Constant parameters (default storage) */
extern const ConstP_RateController_T RateController_ConstP;

/* Class declaration for model RateController */
class RateControllerModelClass {
  /* public data and function members */
 public:
  /* External inputs */
  ExtU_RateController_T RateController_U;

  /* External outputs */
  ExtY_RateController_T RateController_Y;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  RateControllerModelClass();

  /* Destructor */
  ~RateControllerModelClass();

  /* Real-Time Model get method */
  RT_MODEL_RateController_T * getRTM();

  /* private data and function members */
 private:
  /* Block states */
  DW_RateController_T RateController_DW;

  /* Real-Time Model */
  RT_MODEL_RateController_T RateController_M;
};

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
 * '<Root>' : 'RateController'
 * '<S1>'   : 'RateController/Discrete Derivative'
 * '<S2>'   : 'RateController/Discrete Derivative1'
 * '<S3>'   : 'RateController/FD'
 * '<S4>'   : 'RateController/Find Delay'
 * '<S5>'   : 'RateController/INDI_allocator'
 * '<S6>'   : 'RateController/lms'
 * '<S7>'   : 'RateController/Find Delay/Compute Delay'
 * '<S8>'   : 'RateController/Find Delay/Enable Logic'
 */
#endif                                 /* RTW_HEADER_RateController_h_ */
