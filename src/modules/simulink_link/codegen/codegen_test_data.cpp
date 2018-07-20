/*
 * codegen_test_data.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "codegen_test".
 *
 * Model version              : 1.5
 * Simulink Coder version : 8.14 (R2018a) 06-Feb-2018
 * C++ source code generated on : Fri Jul 20 10:15:31 2018
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "codegen_test.h"
#include "codegen_test_private.h"

/* Block parameters (default storage) */
P_codegen_test_T codegen_test_P = {
  /* Mask Parameter: PIDController_D
   * Referenced by: '<S2>/Derivative Gain'
   */
  0.02,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S2>/Integral Gain'
   */
  22.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S2>/Filter Coefficient'
   */
  20.0,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S2>/Proportional Gain'
   */
  1.2,

  /* Expression: InitialConditionForIntegrator
   * Referenced by: '<S2>/Integrator'
   */
  0.0,

  /* Expression: InitialConditionForFilter
   * Referenced by: '<S2>/Filter'
   */
  0.0
};
