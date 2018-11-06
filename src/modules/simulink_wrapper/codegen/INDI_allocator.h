/*
 * INDI_allocator.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "INDI_allocator".
 *
 * Model version              : 1.65
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Tue Nov  6 10:11:18 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_INDI_allocator_h_
#define RTW_HEADER_INDI_allocator_h_
#include <string.h>
#include <cmath>
#include <stddef.h>
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
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T wRotorsensor_DSTATE[4];       /* '<Root>/wRotor sensor' */
  real_T DiscreteStateSpace_DSTATE[3]; /* '<Root>/Discrete State-Space' */
  real_T UD_DSTATE[3];                 /* '<S1>/UD' */
  real_T actuatordynamics_DSTATE[4];   /* '<Root>/actuator dynamics' */
  real_T Memory_PreviousInput[4];      /* '<Root>/Memory' */
} DW_INDI_allocator_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T pqr[3];                       /* '<Root>/pqr' */
  real_T u_pqr[3];                     /* '<Root>/pqr_dot_cmd' */
  real_T thrust_cmd;                   /* '<Root>/thrust_cmd' */
  real_T wRotor[4];                    /* '<Root>/wRotor' */
} ExtU_INDI_allocator_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T w_cmd[4];                     /* '<Root>/w_cmd' */
  real_T w_cmd_px4[4];                 /* '<Root>/w_cmd_px4' */
} ExtY_INDI_allocator_T;

/* Real-time Model Data Structure */
struct tag_RTM_INDI_allocator_T {
  const char_T *errorStatus;
};

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
  /* Block states */
  DW_INDI_allocator_T INDI_allocator_DW;

  /* Real-Time Model */
  RT_MODEL_INDI_allocator_T INDI_allocator_M;
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
 * '<Root>' : 'INDI_allocator'
 * '<S1>'   : 'INDI_allocator/Discrete Derivative'
 * '<S2>'   : 'INDI_allocator/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_INDI_allocator_h_ */
