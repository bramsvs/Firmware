/*
 * RateController_types.h
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

#ifndef RTW_HEADER_RateController_types_h_
#define RTW_HEADER_RateController_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_Kb4703fKot5WPhaNoNlej_
#define DEFINED_TYPEDEF_FOR_struct_Kb4703fKot5WPhaNoNlej_

typedef struct {
  real_T enable;
  real_T maxAngle;
  real_T Kp_pos[3];
  real_T maxVel;
  real_T Kp_vel[3];
  real_T Ki_vel[3];
  real_T intLim;
} struct_Kb4703fKot5WPhaNoNlej;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_7LbCBzsZMvrHROoJfHBYLD_
#define DEFINED_TYPEDEF_FOR_struct_7LbCBzsZMvrHROoJfHBYLD_

typedef struct {
  real_T enable;
  real_T Kp_pos;
  real_T maxVel;
  real_T Kp_vel;
  real_T Ki_vel;
  real_T intLim;
  real_T peakAngle;
  real_T LOCAngle;
  real_T minOmegaSum;
  real_T maxOmegaSum;
} struct_7LbCBzsZMvrHROoJfHBYLD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_0fFeTh16K9OlBqn6rz4Bo_
#define DEFINED_TYPEDEF_FOR_struct_0fFeTh16K9OlBqn6rz4Bo_

typedef struct {
  real_T enable;
  real_T Kp_psi;
  real_T Kp_r;
} struct_0fFeTh16K9OlBqn6rz4Bo;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_XXlBuxXsNoyRaYBWyqwnkB_
#define DEFINED_TYPEDEF_FOR_struct_XXlBuxXsNoyRaYBWyqwnkB_

typedef struct {
  real_T primary_axis[3];
  real_T Iv[9];
  real_T Ip[9];
  real_T t_indi;
  real_T t_rotor_sensor;
  real_T signr;
  real_T failure_id;
  real_T failure_time;
  struct_Kb4703fKot5WPhaNoNlej position;
  struct_7LbCBzsZMvrHROoJfHBYLD altitude;
  struct_0fFeTh16K9OlBqn6rz4Bo YRC;
} struct_XXlBuxXsNoyRaYBWyqwnkB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_T4nMrLe1nFNn3P1xLNPcJE_
#define DEFINED_TYPEDEF_FOR_struct_T4nMrLe1nFNn3P1xLNPcJE_

typedef struct {
  real_T init[16];
  real_T t_filter;
} struct_T4nMrLe1nFNn3P1xLNPcJE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_GVNpqAGGyFxmPFtqvA3EY_
#define DEFINED_TYPEDEF_FOR_struct_GVNpqAGGyFxmPFtqvA3EY_

typedef struct {
  real_T freq;
  real_T g;
  real_T wRotorMax;
  real_T wRotorMin;
  real_T mass;
  real_T failure_id;
  real_T failure_time;
  struct_XXlBuxXsNoyRaYBWyqwnkB sihao;
  struct_T4nMrLe1nFNn3P1xLNPcJE control_eff;
} struct_GVNpqAGGyFxmPFtqvA3EY;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_RateController_T RT_MODEL_RateController_T;

#endif                                 /* RTW_HEADER_RateController_types_h_ */
