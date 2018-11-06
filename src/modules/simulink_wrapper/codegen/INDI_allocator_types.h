/*
 * INDI_allocator_types.h
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

#ifndef RTW_HEADER_INDI_allocator_types_h_
#define RTW_HEADER_INDI_allocator_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_MbZYgFTzRhggMPJGt0Mjb_
#define DEFINED_TYPEDEF_FOR_struct_MbZYgFTzRhggMPJGt0Mjb_

typedef struct {
  real_T primary_axis[3];
  real_T Iv[9];
  real_T Ip[9];
  real_T t_indi;
  real_T t_rotor_sensor;
  real_T signr;
  real_T failure_id;
  real_T failure_time;
} struct_MbZYgFTzRhggMPJGt0Mjb;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_4dVimqcSETJNOUS8r6ij0C_
#define DEFINED_TYPEDEF_FOR_struct_4dVimqcSETJNOUS8r6ij0C_

typedef struct {
  real_T attP;
  real_T attI;
  real_T velLim;
  real_T velP;
  real_T velI;
  real_T MLim;
} struct_4dVimqcSETJNOUS8r6ij0C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_jTdgKoxEUlHmbaNYt1LTCC_
#define DEFINED_TYPEDEF_FOR_struct_jTdgKoxEUlHmbaNYt1LTCC_

typedef struct {
  real_T attP;
  real_T attI;
  real_T velLim;
  real_T velP;
  real_T velI;
  real_T FLim;
} struct_jTdgKoxEUlHmbaNYt1LTCC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_UBKIR2IhXgB4ZvNniEGecB_
#define DEFINED_TYPEDEF_FOR_struct_UBKIR2IhXgB4ZvNniEGecB_

typedef struct {
  real_T k0;
  real_T t0;
  real_T b;
  real_T l;
  real_T mass;
  struct_4dVimqcSETJNOUS8r6ij0C pitch;
  struct_4dVimqcSETJNOUS8r6ij0C roll;
  struct_4dVimqcSETJNOUS8r6ij0C yaw;
  struct_jTdgKoxEUlHmbaNYt1LTCC height;
} struct_UBKIR2IhXgB4ZvNniEGecB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_UP5xrlYk6fdQaGicU0Ko1F_
#define DEFINED_TYPEDEF_FOR_struct_UP5xrlYk6fdQaGicU0Ko1F_

typedef struct {
  real_T freq;
  real_T g;
  real_T wRotorMax;
  real_T wRotorMin;
  struct_MbZYgFTzRhggMPJGt0Mjb sihao;
  struct_UBKIR2IhXgB4ZvNniEGecB simple;
} struct_UP5xrlYk6fdQaGicU0Ko1F;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_INDI_allocator_T RT_MODEL_INDI_allocator_T;

#endif                                 /* RTW_HEADER_INDI_allocator_types_h_ */
