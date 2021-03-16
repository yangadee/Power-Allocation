//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Actuator_Class_Mod_V4.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

#ifndef ACTUATOR_CLASS_MOD_V4_H
#define ACTUATOR_CLASS_MOD_V4_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
class Actuator_Class_Mod_V4 {
public:
  Actuator_Class_Mod_V4 *init();
  void Update_Ta_Matrix(double prv_Ta[24]) const;
  double Lx;
  double Ly;
  double force_max_min[8];
  double angle_max_min[8];
  double delta_angle_max_min[8];
  double servo_maxspeed;
  double dt;
  char angles_out_unit[3];
  double A[4];
  double F[4];
  double T_a[24];
  double Lambda;
  coder::array<double, 1U> X;
  double r;
  double n;
  coder::array<double, 2U> H;
  coder::array<double, 1U> f;
  coder::array<double, 2U> Ceq;
  coder::array<double, 2U> Aieq;
  coder::array<double, 2U> Cieq;

private:
  coder::array<double, 2U> P;
  coder::array<double, 2U> Q;
  coder::array<double, 2U> R;
};

#endif
//
// File trailer for Actuator_Class_Mod_V4.h
//
// [EOF]
//
