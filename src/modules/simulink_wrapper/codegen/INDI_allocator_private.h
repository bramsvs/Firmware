/*
 * INDI_allocator_private.h
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

#ifndef RTW_HEADER_INDI_allocator_private_h_
#define RTW_HEADER_INDI_allocator_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

/* Imported (extern) block parameters */
extern real_T t_indi;                  /* Variable: t_indi
                                        * Referenced by:
                                        *   '<S1>/lowpass_filter '
                                        *   '<S1>/lowpass_filter_indi'
                                        */
extern real_T t_w;                     /* Variable: t_w
                                        * Referenced by:
                                        *   '<S1>/actuator_dynamics1'
                                        *   '<S1>/lowpass_filter_actuator'
                                        */

/* private model entry point functions */
extern void INDI_allocator_derivatives();

#endif                                 /* RTW_HEADER_INDI_allocator_private_h_ */
