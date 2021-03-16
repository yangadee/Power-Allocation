//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Actuator_Class_Mod_V4.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "Actuator_Class_Mod_V4.h"
#include "eye.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <string.h>

// Function Definitions
//
// T_extended
//
// Arguments    : double prv_Ta[24]
// Return Type  : void
//
void Actuator_Class_Mod_V4::Update_Ta_Matrix(double prv_Ta[24]) const
{
  double Lad;
  double b_prv_Ta_tmp;
  double c_prv_Ta_tmp;
  double d_prv_Ta_tmp;
  double e_prv_Ta_tmp;
  double f_prv_Ta_tmp;
  double g_prv_Ta_tmp;
  double h_prv_Ta_tmp;
  double i_prv_Ta_tmp;
  double j_prv_Ta_tmp;
  double k_prv_Ta_tmp;
  double l_prv_Ta_tmp;
  double prv_Lx;
  double prv_Ly;
  double prv_Ta_tmp;
  prv_Lx = this->Lx;
  prv_Ly = this->Ly;
  // 1/ sin cos 45deg
  Lad = this->Lambda;
  //  lambda
  prv_Ta_tmp = std::cos(this->A[1]);
  b_prv_Ta_tmp = std::cos(this->A[3]);
  c_prv_Ta_tmp = std::sin(this->A[0]);
  d_prv_Ta_tmp = std::sin(this->A[1]);
  e_prv_Ta_tmp = std::sin(this->A[2]);
  f_prv_Ta_tmp = std::sin(this->A[3]);
  g_prv_Ta_tmp = std::cos(this->A[0]);
  h_prv_Ta_tmp = std::cos(this->A[2]);
  prv_Ta[0] = 1.4142135623730949 * g_prv_Ta_tmp;
  prv_Ta[6] = 1.4142135623730949 * prv_Ta_tmp;
  prv_Ta[12] = 1.4142135623730949 * h_prv_Ta_tmp;
  prv_Ta[18] = 1.4142135623730949 * b_prv_Ta_tmp;
  prv_Ta[1] = 1.4142135623730949 * std::cos(this->A[0]);
  prv_Ta[7] = -1.4142135623730949 * prv_Ta_tmp;
  prv_Ta[13] = 1.4142135623730949 * std::cos(this->A[2]);
  prv_Ta[19] = -1.4142135623730949 * b_prv_Ta_tmp;
  prv_Ta[2] = -c_prv_Ta_tmp;
  prv_Ta[8] = -d_prv_Ta_tmp;
  prv_Ta[14] = -e_prv_Ta_tmp;
  prv_Ta[20] = -f_prv_Ta_tmp;
  i_prv_Ta_tmp = std::cos(this->A[0]) * 1.4142135623730949 * Lad;
  prv_Ta[3] = prv_Ly * c_prv_Ta_tmp - i_prv_Ta_tmp;
  j_prv_Ta_tmp = std::cos(this->A[1]) * 1.4142135623730949 * Lad;
  prv_Ta[9] = -prv_Ly * d_prv_Ta_tmp + j_prv_Ta_tmp;
  k_prv_Ta_tmp = std::cos(this->A[2]) * 1.4142135623730949 * Lad;
  prv_Ta[15] = -prv_Ly * e_prv_Ta_tmp - k_prv_Ta_tmp;
  l_prv_Ta_tmp = std::cos(this->A[3]) * 1.4142135623730949 * Lad;
  prv_Ta[21] = prv_Ly * f_prv_Ta_tmp + l_prv_Ta_tmp;
  prv_Ta[4] = prv_Lx * c_prv_Ta_tmp - i_prv_Ta_tmp;
  prv_Ta[10] = prv_Lx * d_prv_Ta_tmp - j_prv_Ta_tmp;
  prv_Ta[16] = -prv_Lx * e_prv_Ta_tmp - k_prv_Ta_tmp;
  prv_Ta[22] = -prv_Lx * f_prv_Ta_tmp - l_prv_Ta_tmp;
  i_prv_Ta_tmp = prv_Lx + prv_Ly;
  j_prv_Ta_tmp = i_prv_Ta_tmp * 1.4142135623730949;
  prv_Ta[5] = j_prv_Ta_tmp * g_prv_Ta_tmp + c_prv_Ta_tmp * Lad;
  c_prv_Ta_tmp = -i_prv_Ta_tmp * 1.4142135623730949;
  prv_Ta[11] = c_prv_Ta_tmp * prv_Ta_tmp - d_prv_Ta_tmp * Lad;
  prv_Ta[17] = c_prv_Ta_tmp * h_prv_Ta_tmp + e_prv_Ta_tmp * Lad;
  prv_Ta[23] = j_prv_Ta_tmp * b_prv_Ta_tmp - f_prv_Ta_tmp * Lad;
}

//
// Perform one-time calculations, such as computing constants
//
// Arguments    : void
// Return Type  : Actuator_Class_Mod_V4 *
//
Actuator_Class_Mod_V4 *Actuator_Class_Mod_V4::init()
{
  static const double dv[8]{3.1415926535897931,  3.1415926535897931,
                            3.1415926535897931,  3.1415926535897931,
                            -3.1415926535897931, -3.1415926535897931,
                            -3.1415926535897931, -3.1415926535897931};
  static const signed char iv1[16]{5, 0, 0, 0, 0, 5, 0, 0,
                                   0, 0, 5, 0, 0, 0, 0, 5};
  static const unsigned char uv[16]{150U, 0U, 0U,   0U, 0U, 150U, 0U, 0U,
                                    0U,   0U, 150U, 0U, 0U, 0U,   0U, 150U};
  static const signed char iv[8]{50, 50, 50, 50, -10, -10, -10, -10};
  Actuator_Class_Mod_V4 *obj;
  coder::array<double, 2U> *C_tmp;
  coder::array<double, 2U> b;
  coder::array<double, 2U> b_result;
  coder::array<double, 2U> b_varargin_1;
  coder::array<double, 2U> b_varargin_3;
  coder::array<double, 2U> d_result;
  coder::array<double, 2U> e_result;
  coder::array<double, 2U> result;
  coder::array<double, 2U> varargin_1;
  coder::array<double, 2U> varargin_2;
  coder::array<signed char, 2U> varargin_3;
  double dv1[24];
  double b_y;
  double y;
  int b_i;
  int boffset;
  int c_result;
  int coffset;
  int i;
  int i1;
  int input_sizes_idx_0;
  int input_sizes_idx_1;
  int sizes_idx_0;
  boolean_T empty_non_axis_sizes;
  obj = this;
  obj->angles_out_unit[0] = 'd';
  obj->angles_out_unit[1] = 'e';
  obj->angles_out_unit[2] = 'g';
  obj->Lambda = 0.1;
  obj->r = 4.0;
  obj->n = 6.0;
  obj->dt = 0.01;
  obj->Lx = 0.25;
  obj->Ly = 0.25;
  obj->servo_maxspeed = 3.4906585039886591;
  y = -obj->servo_maxspeed * obj->dt;
  b_y = obj->servo_maxspeed * obj->dt;
  for (i = 0; i < 8; i++) {
    obj->force_max_min[i] = iv[i];
  }
  for (i = 0; i < 8; i++) {
    obj->angle_max_min[i] = dv[i];
  }
  obj->delta_angle_max_min[0] = b_y;
  obj->delta_angle_max_min[1] = b_y;
  obj->delta_angle_max_min[2] = b_y;
  obj->delta_angle_max_min[3] = b_y;
  obj->delta_angle_max_min[4] = y;
  obj->delta_angle_max_min[5] = y;
  obj->delta_angle_max_min[6] = y;
  obj->delta_angle_max_min[7] = y;
  obj->A[0] = 1.5707963267948966;
  obj->A[1] = 1.5707963267948966;
  obj->A[2] = 1.5707963267948966;
  obj->A[3] = 1.5707963267948966;
  // zeros(4,1);          % initial angles
  //              obj.A(1) =  0*pi*ones(1,1);
  //              obj.A(2) =  0.5*pi*ones(1,1);
  //              obj.A(3) = -.5*pi*ones(1,1);
  //              obj.A(4) =  -.5*pi*ones(1,1);
  obj->F[0] = 0.0;
  obj->F[1] = 0.0;
  obj->F[2] = 0.0;
  obj->F[3] = 0.0;
  //  initial forces
  obj->Update_Ta_Matrix(dv1);
  std::copy(&dv1[0], &dv1[24], &obj->T_a[0]);
  coder::eye(obj->r, b);
  C_tmp = &obj->P;
  input_sizes_idx_0 = b.size(1);
  obj->P.set_size(4, b.size(1));
  for (input_sizes_idx_1 = 0; input_sizes_idx_1 < input_sizes_idx_0;
       input_sizes_idx_1++) {
    coffset = input_sizes_idx_1 << 2;
    boffset = input_sizes_idx_1 * b.size(0);
    for (i = 0; i < 4; i++) {
      (*C_tmp)[coffset + i] =
          ((static_cast<double>(uv[i]) * b[boffset] +
            static_cast<double>(uv[i + 4]) * b[boffset + 1]) +
           static_cast<double>(uv[i + 8]) * b[boffset + 2]) +
          static_cast<double>(uv[i + 12]) * b[boffset + 3];
    }
  }
  //  f % r x r = 4x4
  coder::eye(obj->r, b);
  C_tmp = &obj->Q;
  input_sizes_idx_0 = b.size(1);
  obj->Q.set_size(4, b.size(1));
  for (input_sizes_idx_1 = 0; input_sizes_idx_1 < input_sizes_idx_0;
       input_sizes_idx_1++) {
    coffset = input_sizes_idx_1 << 2;
    boffset = input_sizes_idx_1 * b.size(0);
    for (i = 0; i < 4; i++) {
      (*C_tmp)[coffset + i] =
          ((static_cast<double>(iv1[i]) * b[boffset] +
            static_cast<double>(iv1[i + 4]) * b[boffset + 1]) +
           static_cast<double>(iv1[i + 8]) * b[boffset + 2]) +
          static_cast<double>(iv1[i + 12]) * b[boffset + 3];
    }
  }
  //  f % r x r = 12x12
  coder::eye(obj->n, b);
  coder::internal::blas::mtimes(b, obj->R);
  //  Tau % equal to size of Tau_in = nxn = 6x6
  // Hessian matrix combined P, Q and R
  varargin_1.set_size(4, obj->P.size(1));
  input_sizes_idx_0 = 4 * obj->P.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_1[b_i] = obj->P[b_i];
  }
  varargin_2.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = 0.0;
  }
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if (varargin_1.size(1) != 0) {
    sizes_idx_0 = 4;
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    sizes_idx_0 = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    sizes_idx_0 = varargin_3.size(0);
  } else {
    sizes_idx_0 = 4;
  }
  if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    boffset = varargin_2.size(1);
  } else {
    boffset = 0;
  }
  if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  if (varargin_1.size(1) != 0) {
    input_sizes_idx_0 = varargin_1.size(1);
  } else {
    input_sizes_idx_0 = 0;
  }
  if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  b.set_size(sizes_idx_0, (input_sizes_idx_0 + boffset) + i);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      b[i1 + b.size(0) * b_i] = varargin_1[i1 + sizes_idx_0 * b_i];
    }
  }
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      b[i1 + b.size(0) * (b_i + input_sizes_idx_0)] = 0.0;
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      b[i1 + b.size(0) * ((b_i + input_sizes_idx_0) + input_sizes_idx_1)] = 0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  varargin_1.set_size(4, obj->Q.size(1));
  input_sizes_idx_0 = 4 * obj->Q.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_1[b_i] = obj->Q[b_i];
  }
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    sizes_idx_0 = b_varargin_1.size(0);
  } else if (varargin_1.size(1) != 0) {
    sizes_idx_0 = 4;
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    sizes_idx_0 = varargin_3.size(0);
  } else {
    sizes_idx_0 = 4;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (varargin_1.size(1) != 0) {
    coffset = varargin_1.size(1);
  } else {
    coffset = 0;
  }
  if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    input_sizes_idx_0 = b_varargin_1.size(1);
  } else {
    input_sizes_idx_0 = 0;
  }
  if (varargin_1.size(1) != 0) {
    input_sizes_idx_1 = varargin_1.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  result.set_size(sizes_idx_0, (boffset + coffset) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      result[i1 + result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < coffset; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      result[i1 + result.size(0) * (b_i + input_sizes_idx_0)] =
          varargin_1[i1 + sizes_idx_0 * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      result[i1 + result.size(0) *
                      ((b_i + input_sizes_idx_0) + input_sizes_idx_1)] = 0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->n), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->n) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  varargin_2.set_size(static_cast<int>(obj->n), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->n) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = 0.0;
  }
  b_varargin_3.set_size(6, obj->R.size(1));
  input_sizes_idx_0 = 6 * obj->R.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_3[b_i] = obj->R[b_i];
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    sizes_idx_0 = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    sizes_idx_0 = varargin_2.size(0);
  } else if (b_varargin_3.size(1) != 0) {
    sizes_idx_0 = 6;
  } else {
    sizes_idx_0 = 6;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    coffset = varargin_2.size(1);
  } else {
    coffset = 0;
  }
  if (b_varargin_3.size(1) != 0) {
    i = b_varargin_3.size(1);
  } else {
    i = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    input_sizes_idx_0 = b_varargin_1.size(1);
  } else {
    input_sizes_idx_0 = 0;
  }
  if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  b_result.set_size(sizes_idx_0, (boffset + coffset) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      b_result[i1 + b_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < coffset; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      b_result[i1 + b_result.size(0) * (b_i + input_sizes_idx_0)] = 0.0;
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      b_result[i1 + b_result.size(0) *
                        ((b_i + input_sizes_idx_0) + input_sizes_idx_1)] =
          b_varargin_3[i1 + sizes_idx_0 * b_i];
    }
  }
  if (b.size(1) != 0) {
    c_result = b.size(1);
  } else if (result.size(1) != 0) {
    c_result = result.size(1);
  } else if (b_result.size(1) != 0) {
    c_result = b_result.size(1);
  } else {
    c_result = 0;
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes || (b.size(1) != 0)) {
    input_sizes_idx_0 = b.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }
  if (empty_non_axis_sizes || (result.size(1) != 0)) {
    coffset = result.size(0);
  } else {
    coffset = 0;
  }
  if (empty_non_axis_sizes || (b_result.size(1) != 0)) {
    sizes_idx_0 = b_result.size(0);
  } else {
    sizes_idx_0 = 0;
  }
  obj->H.set_size((input_sizes_idx_0 + coffset) + sizes_idx_0, c_result);
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
      obj->H[i1 + obj->H.size(0) * b_i] = b[i1 + input_sizes_idx_0 * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < coffset; i1++) {
      obj->H[(i1 + input_sizes_idx_0) + obj->H.size(0) * b_i] =
          result[i1 + coffset * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      obj->H[((i1 + input_sizes_idx_0) + coffset) + obj->H.size(0) * b_i] =
          b_result[i1 + sizes_idx_0 * b_i];
    }
  }
  //  note: this is not control forces f, it is  linear multiplier vector, f = 0
  //  matrix in this case
  input_sizes_idx_0 = static_cast<int>(2.0 * obj->r + obj->n);
  obj->f.set_size(input_sizes_idx_0);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    obj->f[b_i] = 0.0;
  }
  //  equality constrains
  coder::eye(obj->n, b_varargin_1);
  varargin_2.set_size(static_cast<int>(obj->n), static_cast<int>(6.0 * obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->n) * static_cast<int>(6.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = 0.0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    i = varargin_2.size(1);
  } else {
    i = 0;
  }
  obj->Ceq.set_size(c_result, boffset + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      obj->Ceq[i1 + obj->Ceq.size(0) * b_i] = b_varargin_1[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      obj->Ceq[i1 + obj->Ceq.size(0) * (b_i + boffset)] = 0.0;
    }
  }
  //  inequality constrains
  coder::eye(obj->r, b_varargin_1);
  varargin_2.set_size(static_cast<int>(obj->r),
                      static_cast<int>(obj->n + obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = 0.0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    i = varargin_2.size(1);
  } else {
    i = 0;
  }
  b.set_size(c_result, boffset + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b[i1 + b.size(0) * b_i] = b_varargin_1[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b[i1 + b.size(0) * (b_i + boffset)] = 0.0;
    }
  }
  coder::eye(obj->r, b_varargin_1);
  input_sizes_idx_0 = b_varargin_1.size(0) * b_varargin_1.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = -b_varargin_1[b_i];
  }
  varargin_2.set_size(static_cast<int>(obj->r),
                      static_cast<int>(obj->n + obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = 0.0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    i = varargin_2.size(1);
  } else {
    i = 0;
  }
  result.set_size(c_result, boffset + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      result[i1 + result.size(0) * b_i] = b_varargin_1[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      result[i1 + result.size(0) * (b_i + boffset)] = 0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  b_result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_result[i1 + b_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_result[i1 + b_result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_result[i1 + b_result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] =
          0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  input_sizes_idx_0 = varargin_2.size(0) * varargin_2.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = -varargin_2[b_i];
  }
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  d_result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      d_result[i1 + d_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      d_result[i1 + d_result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      d_result[i1 + d_result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] =
          0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  e_result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      e_result[i1 + e_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      e_result[i1 + e_result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      e_result[i1 + e_result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] =
          0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  input_sizes_idx_0 = varargin_2.size(0) * varargin_2.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = -varargin_2[b_i];
  }
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  b_varargin_1.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_varargin_1[i1 + b_varargin_1.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_varargin_1[i1 + b_varargin_1.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_varargin_1[i1 + b_varargin_1.size(0) *
                            ((b_i + boffset) + input_sizes_idx_1)] = 0.0;
    }
  }
  if ((b.size(0) != 0) && (b.size(1) != 0)) {
    c_result = b.size(1);
  } else if ((result.size(0) != 0) && (result.size(1) != 0)) {
    c_result = result.size(1);
  } else if ((b_result.size(0) != 0) && (b_result.size(1) != 0)) {
    c_result = b_result.size(1);
  } else if ((d_result.size(0) != 0) && (d_result.size(1) != 0)) {
    c_result = d_result.size(1);
  } else if ((e_result.size(0) != 0) && (e_result.size(1) != 0)) {
    c_result = e_result.size(1);
  } else if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(1);
  } else {
    c_result = b.size(1);
    if (result.size(1) > b.size(1)) {
      c_result = result.size(1);
    }
    if (b_result.size(1) > c_result) {
      c_result = b_result.size(1);
    }
    if (d_result.size(1) > c_result) {
      c_result = d_result.size(1);
    }
    if (e_result.size(1) > c_result) {
      c_result = e_result.size(1);
    }
    if (b_varargin_1.size(1) > c_result) {
      c_result = b_varargin_1.size(1);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes || ((b.size(0) != 0) && (b.size(1) != 0))) {
    input_sizes_idx_0 = b.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }
  if (empty_non_axis_sizes ||
      ((result.size(0) != 0) && (result.size(1) != 0))) {
    coffset = result.size(0);
  } else {
    coffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((b_result.size(0) != 0) && (b_result.size(1) != 0))) {
    boffset = b_result.size(0);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((d_result.size(0) != 0) && (d_result.size(1) != 0))) {
    i = d_result.size(0);
  } else {
    i = 0;
  }
  if (empty_non_axis_sizes ||
      ((e_result.size(0) != 0) && (e_result.size(1) != 0))) {
    input_sizes_idx_1 = e_result.size(0);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    sizes_idx_0 = b_varargin_1.size(0);
  } else {
    sizes_idx_0 = 0;
  }
  obj->Aieq.set_size(
      ((((input_sizes_idx_0 + coffset) + boffset) + i) + input_sizes_idx_1) +
          sizes_idx_0,
      c_result);
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
      obj->Aieq[i1 + obj->Aieq.size(0) * b_i] = b[i1 + input_sizes_idx_0 * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < coffset; i1++) {
      obj->Aieq[(i1 + input_sizes_idx_0) + obj->Aieq.size(0) * b_i] =
          result[i1 + coffset * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < boffset; i1++) {
      obj->Aieq[((i1 + input_sizes_idx_0) + coffset) +
                obj->Aieq.size(0) * b_i] = b_result[i1 + boffset * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < i; i1++) {
      obj->Aieq[(((i1 + input_sizes_idx_0) + coffset) + boffset) +
                obj->Aieq.size(0) * b_i] = d_result[i1 + i * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      obj->Aieq[((((i1 + input_sizes_idx_0) + coffset) + boffset) + i) +
                obj->Aieq.size(0) * b_i] =
          e_result[i1 + input_sizes_idx_1 * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      obj->Aieq[(((((i1 + input_sizes_idx_0) + coffset) + boffset) + i) +
                 input_sizes_idx_1) +
                obj->Aieq.size(0) * b_i] = b_varargin_1[i1 + sizes_idx_0 * b_i];
    }
  }
  // 6r x (n+3r)
  b_varargin_1.set_size(static_cast<int>(obj->r), static_cast<int>(obj->n));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->n);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(5.0 * obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(5.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  b.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b[i1 + b.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b[i1 + b.size(0) * (b_i + boffset)] = varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b[i1 + b.size(0) * ((b_i + boffset) + input_sizes_idx_1)] = 0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r),
                        static_cast<int>(obj->n + obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  input_sizes_idx_0 = varargin_2.size(0) * varargin_2.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = -varargin_2[b_i];
  }
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(4.0 * obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(4.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      result[i1 + result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      result[i1 + result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      result[i1 + result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] = 0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r),
                        static_cast<int>(obj->n + 2.0 * obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + 2.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(3.0 * obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(3.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  b_result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_result[i1 + b_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_result[i1 + b_result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_result[i1 + b_result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] =
          0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r),
                        static_cast<int>(obj->n + 3.0 * obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + 3.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  input_sizes_idx_0 = varargin_2.size(0) * varargin_2.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = -varargin_2[b_i];
  }
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(2.0 * obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(2.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  d_result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      d_result[i1 + d_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      d_result[i1 + d_result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      d_result[i1 + d_result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] =
          0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r),
                        static_cast<int>(obj->n + 4.0 * obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + 4.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  varargin_3.set_size(static_cast<int>(obj->r), static_cast<int>(obj->r));
  input_sizes_idx_0 = static_cast<int>(obj->r) * static_cast<int>(obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_3[b_i] = 0;
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else if ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0)) {
    c_result = varargin_3.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
    if (varargin_3.size(0) > c_result) {
      c_result = varargin_3.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    input_sizes_idx_1 = varargin_2.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_3.size(0) != 0) && (varargin_3.size(1) != 0))) {
    i = varargin_3.size(1);
  } else {
    i = 0;
  }
  e_result.set_size(c_result, (boffset + input_sizes_idx_1) + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      e_result[i1 + e_result.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < input_sizes_idx_1; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      e_result[i1 + e_result.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      e_result[i1 + e_result.size(0) * ((b_i + boffset) + input_sizes_idx_1)] =
          0.0;
    }
  }
  b_varargin_1.set_size(static_cast<int>(obj->r),
                        static_cast<int>(obj->n + 5.0 * obj->r));
  input_sizes_idx_0 =
      static_cast<int>(obj->r) * static_cast<int>(obj->n + 5.0 * obj->r);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    b_varargin_1[b_i] = 0.0;
  }
  coder::eye(obj->r, varargin_2);
  input_sizes_idx_0 = varargin_2.size(0) * varargin_2.size(1);
  for (b_i = 0; b_i < input_sizes_idx_0; b_i++) {
    varargin_2[b_i] = -varargin_2[b_i];
  }
  if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(0);
  } else if ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0)) {
    c_result = varargin_2.size(0);
  } else {
    c_result = b_varargin_1.size(0);
    if (varargin_2.size(0) > b_varargin_1.size(0)) {
      c_result = varargin_2.size(0);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    boffset = b_varargin_1.size(1);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((varargin_2.size(0) != 0) && (varargin_2.size(1) != 0))) {
    i = varargin_2.size(1);
  } else {
    i = 0;
  }
  b_varargin_1.set_size(c_result, boffset + i);
  for (b_i = 0; b_i < boffset; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_varargin_1[i1 + b_varargin_1.size(0) * b_i] = 0.0;
    }
  }
  for (b_i = 0; b_i < i; b_i++) {
    for (i1 = 0; i1 < c_result; i1++) {
      b_varargin_1[i1 + b_varargin_1.size(0) * (b_i + boffset)] =
          varargin_2[i1 + c_result * b_i];
    }
  }
  if ((b.size(0) != 0) && (b.size(1) != 0)) {
    c_result = b.size(1);
  } else if ((result.size(0) != 0) && (result.size(1) != 0)) {
    c_result = result.size(1);
  } else if ((b_result.size(0) != 0) && (b_result.size(1) != 0)) {
    c_result = b_result.size(1);
  } else if ((d_result.size(0) != 0) && (d_result.size(1) != 0)) {
    c_result = d_result.size(1);
  } else if ((e_result.size(0) != 0) && (e_result.size(1) != 0)) {
    c_result = e_result.size(1);
  } else if ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0)) {
    c_result = b_varargin_1.size(1);
  } else {
    c_result = b.size(1);
    if (result.size(1) > b.size(1)) {
      c_result = result.size(1);
    }
    if (b_result.size(1) > c_result) {
      c_result = b_result.size(1);
    }
    if (d_result.size(1) > c_result) {
      c_result = d_result.size(1);
    }
    if (e_result.size(1) > c_result) {
      c_result = e_result.size(1);
    }
    if (b_varargin_1.size(1) > c_result) {
      c_result = b_varargin_1.size(1);
    }
  }
  empty_non_axis_sizes = (c_result == 0);
  if (empty_non_axis_sizes || ((b.size(0) != 0) && (b.size(1) != 0))) {
    input_sizes_idx_0 = b.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }
  if (empty_non_axis_sizes ||
      ((result.size(0) != 0) && (result.size(1) != 0))) {
    coffset = result.size(0);
  } else {
    coffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((b_result.size(0) != 0) && (b_result.size(1) != 0))) {
    boffset = b_result.size(0);
  } else {
    boffset = 0;
  }
  if (empty_non_axis_sizes ||
      ((d_result.size(0) != 0) && (d_result.size(1) != 0))) {
    i = d_result.size(0);
  } else {
    i = 0;
  }
  if (empty_non_axis_sizes ||
      ((e_result.size(0) != 0) && (e_result.size(1) != 0))) {
    input_sizes_idx_1 = e_result.size(0);
  } else {
    input_sizes_idx_1 = 0;
  }
  if (empty_non_axis_sizes ||
      ((b_varargin_1.size(0) != 0) && (b_varargin_1.size(1) != 0))) {
    sizes_idx_0 = b_varargin_1.size(0);
  } else {
    sizes_idx_0 = 0;
  }
  obj->Cieq.set_size(
      ((((input_sizes_idx_0 + coffset) + boffset) + i) + input_sizes_idx_1) +
          sizes_idx_0,
      c_result);
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < input_sizes_idx_0; i1++) {
      obj->Cieq[i1 + obj->Cieq.size(0) * b_i] = b[i1 + input_sizes_idx_0 * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < coffset; i1++) {
      obj->Cieq[(i1 + input_sizes_idx_0) + obj->Cieq.size(0) * b_i] =
          result[i1 + coffset * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < boffset; i1++) {
      obj->Cieq[((i1 + input_sizes_idx_0) + coffset) +
                obj->Cieq.size(0) * b_i] = b_result[i1 + boffset * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < i; i1++) {
      obj->Cieq[(((i1 + input_sizes_idx_0) + coffset) + boffset) +
                obj->Cieq.size(0) * b_i] = d_result[i1 + i * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      obj->Cieq[((((i1 + input_sizes_idx_0) + coffset) + boffset) + i) +
                obj->Cieq.size(0) * b_i] =
          e_result[i1 + input_sizes_idx_1 * b_i];
    }
  }
  for (b_i = 0; b_i < c_result; b_i++) {
    for (i1 = 0; i1 < sizes_idx_0; i1++) {
      obj->Cieq[(((((i1 + input_sizes_idx_0) + coffset) + boffset) + i) +
                 input_sizes_idx_1) +
                obj->Cieq.size(0) * b_i] = b_varargin_1[i1 + sizes_idx_0 * b_i];
    }
  }
  // 6r x (n+6r)
  return obj;
}

//
// File trailer for Actuator_Class_Mod_V4.cpp
//
// [EOF]
//
