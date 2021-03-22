//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Test_Actuator_Mod_V4.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "Test_Actuator_Mod_V4.h"
#include "Actuator_Class_Mod_V4.h"
#include "Test_Actuator_Mod_V4_data.h"
#include "Test_Actuator_Mod_V4_initialize.h"
#include "fileManager.h"
#include "qpkwik.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Function Declarations
static int div_nde_s32_ceiling(int numerator, int denominator);

// Function Definitions
//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_nde_s32_ceiling(int numerator, int denominator)
{
  return numerator / denominator + (((numerator < 0) == (denominator < 0)) &&
                                    (numerator % denominator != 0));
}

//
// Control allocation for angle and force output
//  Author:   ADE
//  Date:     20210122
//  Revisions:
//  function Local_Convex_QP()
//  Note:
//            Tau_in is control force by guidance law in matrix format with
//            matrix size = nx1 T_ext is extended thrust configuration matrix
//            which define by user
//
//
//  QP form: J= x'Hx + f'x
//                        Aieq*x <= Bieq
//                        Aeq*x   = Beq
//  in this function x = combination of Force_out and slack variable s
//                                      where s has the same size of Tau_in
//  function:      [x,exitflag,iA,lambda] =
//  mpcActiveSetSolver(H,f,Aieq,bieq,Aeq,beq,iA0,options)
//
// Arguments    : void
// Return Type  : void
//
void Test_Actuator_Mod_V4()
{
  static const char b[3]{'d', 'e', 'g'};
  static const char b_b[3]{'r', 'a', 'd'};
  FILE *b_NULL;
  FILE *filestar;
  size_t nBytes;
  Actuator_Class_Mod_V4 AC;
  coder::array<double, 2U> Aeq;
  coder::array<double, 2U> At;
  coder::array<double, 2U> Tau_in;
  coder::array<double, 2U> b_a;
  coder::array<double, 2U> c_a;
  coder::array<double, 2U> d_a;
  coder::array<double, 1U> Bieq;
  coder::array<double, 1U> Dieq;
  coder::array<double, 1U> b_Bieq;
  coder::array<double, 1U> lam;
  coder::array<double, 1U> r;
  coder::array<short, 1U> iA1;
  coder::array<signed char, 2U> b_I;
  double tbuf[1024];
  double p_data[30];
  double dv[24];
  double Act_Out[14];
  double s_data[6];
  double prv_Lx;
  int boffset;
  int bytesOut;
  int c;
  int exponent;
  int i;
  int i1;
  int i2;
  int k;
  int loop_ub;
  int numRead;
  signed char fileid;
  boolean_T a;
  if (!isInitialized_Test_Actuator_Mod_V4) {
    Test_Actuator_Mod_V4_initialize();
  }
  //  QP crition: J = f'Wf + (f-f0)'M(f-f0)+ s'Qs
  //  x = [f, da, s]
  //  load Tau.mat                  %控制力 (Utau)
  // %% Load bin file
  //  % %To Save bin file
  //  FID = fopen(FileName, 'w');
  //  fwrite(FID, Utau, 'double');
  //  fclose(FID);
  //  % %To load Save bin file
  fileid = coder::cfopen("Utau3.dat", "rb");
  nBytes = sizeof(double);
  coder::getfilestar(static_cast<double>(fileid), &filestar, &a);
  if ((fileid == 0) || (fileid == 1) || (fileid == 2)) {
    filestar = NULL;
  }
  At.set_size(0, 1);
  if (filestar == NULL) {
    bytesOut = -1;
  } else {
    c = 1;
    bytesOut = -1;
    while (c > 0) {
      c = 0;
      numRead = 1;
      while ((c < 1024) && (numRead > 0)) {
        size_t numReadSizeT;
        numReadSizeT = fread(&tbuf[c], nBytes, 1024 - c, filestar);
        numRead = (int)numReadSizeT;
        c += (int)numReadSizeT;
      }
      if (1 > c) {
        boffset = 0;
      } else {
        boffset = c;
      }
      Dieq.set_size(At.size(0) + boffset);
      numRead = At.size(0);
      for (i = 0; i < numRead; i++) {
        Dieq[i] = At[i];
      }
      for (i = 0; i < boffset; i++) {
        Dieq[i + At.size(0)] = tbuf[i];
      }
      At.set_size(Dieq.size(0), 1);
      boffset = Dieq.size(0);
      for (i = 0; i < boffset; i++) {
        At[i] = Dieq[i];
      }
      bytesOut += c;
    }
  }
  if (At.size(0) >= 6) {
    Tau_in.set_size(6, div_nde_s32_ceiling(bytesOut + 1, 6));
    for (k = 0; k <= bytesOut; k++) {
      Tau_in[k] = At[k];
    }
    i = bytesOut + 2;
    i1 = 6 * Tau_in.size(1);
    for (k = i; k <= i1; k++) {
      Tau_in[k - 1] = 0.0;
    }
  } else {
    Tau_in.set_size(At.size(0), 1);
    boffset = At.size(0);
    for (i = 0; i < boffset; i++) {
      Tau_in[i] = At[i];
    }
  }
  coder::cfclose(static_cast<double>(fileid));
  AC.init();
  //  for results Output
  fileid = coder::cfopen("ResultOut1.txt", "wb");
  coder::cfclose(static_cast<double>(fileid));
  fileid = coder::cfopen("ResultOut1.txt", "ab");
  i = Tau_in.size(1);
  if (0 <= Tau_in.size(1) - 1) {
    i2 = Tau_in.size(0);
    loop_ub = Tau_in.size(0);
    b_NULL = NULL;
  }
  for (int b_i{0}; b_i < i; b_i++) {
    double Force_Out_idx_1;
    double Lad;
    double b_prv_Jcb_Tau_tmp;
    double c_prv_Jcb_Tau_tmp;
    double d_prv_Jcb_Tau_tmp;
    double e_prv_Jcb_Tau_tmp;
    double f_prv_Jcb_Tau_tmp;
    double g_prv_Jcb_Tau_tmp;
    double h_prv_Jcb_Tau_tmp;
    double prv_A_idx_0;
    double prv_A_idx_1;
    double prv_A_idx_2;
    double prv_A_idx_3;
    double prv_Jcb_Tau_tmp;
    double prv_Ly;
    double t;
    int c_i;
    int inner;
    int mc;
    int sizes_idx_1;
    short i3;
    // tic
    for (i1 = 0; i1 < loop_ub; i1++) {
      p_data[i1] = Tau_in[i1 + Tau_in.size(0) * b_i];
    }
    std::copy(&AC.force_max_min[0], &AC.force_max_min[8], &p_data[i2]);
    std::copy(&AC.angle_max_min[0], &AC.angle_max_min[8], &p_data[i2 + 8]);
    std::copy(&AC.delta_angle_max_min[0], &AC.delta_angle_max_min[8],
              &p_data[i2 + 16]);
    AC.Update_Ta_Matrix(dv);
    std::copy(&dv[0], &dv[24], &AC.T_a[0]);
    //  Tau = Ta x F
    prv_Lx = AC.Lx;
    prv_Ly = AC.Ly;
    prv_A_idx_0 = AC.A[0];
    prv_A_idx_1 = AC.A[1];
    prv_A_idx_2 = AC.A[2];
    prv_A_idx_3 = AC.A[3];
    // 1/ sin cos 45deg
    Lad = AC.Lambda;
    //  lambda
    prv_Jcb_Tau_tmp = std::sin(prv_A_idx_1);
    b_prv_Jcb_Tau_tmp = std::sin(prv_A_idx_3);
    c_prv_Jcb_Tau_tmp = std::cos(prv_A_idx_0);
    d_prv_Jcb_Tau_tmp = std::sin(prv_A_idx_0);
    e_prv_Jcb_Tau_tmp = std::cos(prv_A_idx_1);
    f_prv_Jcb_Tau_tmp = std::cos(prv_A_idx_2);
    g_prv_Jcb_Tau_tmp = std::sin(prv_A_idx_2);
    h_prv_Jcb_Tau_tmp = std::cos(prv_A_idx_3);
    //  equality constrains
    Force_Out_idx_1 = AC.n;
    if (Force_Out_idx_1 < 0.0) {
      t = 0.0;
      numRead = 0;
    } else {
      t = Force_Out_idx_1;
      numRead = static_cast<int>(Force_Out_idx_1);
    }
    b_I.set_size(numRead, numRead);
    boffset = static_cast<int>(t) * static_cast<int>(t);
    for (i1 = 0; i1 < boffset; i1++) {
      b_I[i1] = 0;
    }
    if (static_cast<int>(t) > 0) {
      for (k = 0; k < numRead; k++) {
        b_I[k + b_I.size(0) * k] = 1;
      }
    }
    boffset = b_I.size(0) * b_I.size(1);
    for (i1 = 0; i1 < boffset; i1++) {
      b_I[i1] = static_cast<signed char>(-b_I[i1]);
    }
    if ((b_I.size(0) != 0) && (b_I.size(1) != 0)) {
      bytesOut = b_I.size(1);
    } else {
      bytesOut = 0;
    }
    // double(subs(Jcb_Taf))
    b_a.set_size(AC.Ceq.size(0), AC.Ceq.size(1));
    boffset = AC.Ceq.size(0) * AC.Ceq.size(1);
    for (i1 = 0; i1 < boffset; i1++) {
      b_a[i1] = AC.Ceq[i1];
    }
    //  inequality constrains
    numRead = static_cast<int>(2.0 * AC.r);
    c = static_cast<int>(2.0 * AC.r);
    Dieq.set_size((numRead + c) + 8);
    for (i1 = 0; i1 < numRead; i1++) {
      Dieq[i1] = 0.0;
    }
    Dieq[numRead] = -AC.A[0];
    Dieq[numRead + 1] = -AC.A[1];
    Dieq[numRead + 2] = -AC.A[2];
    Dieq[numRead + 3] = -AC.A[3];
    Dieq[numRead + 4] = AC.A[0];
    Dieq[numRead + 5] = AC.A[1];
    Dieq[numRead + 6] = AC.A[2];
    Dieq[numRead + 7] = AC.A[3];
    for (i1 = 0; i1 < c; i1++) {
      Dieq[(i1 + numRead) + 8] = 0.0;
    }
    c_a.set_size(AC.Cieq.size(0), AC.Cieq.size(1));
    boffset = AC.Cieq.size(0) * AC.Cieq.size(1);
    for (i1 = 0; i1 < boffset; i1++) {
      c_a[i1] = AC.Cieq[i1];
    }
    mc = c_a.size(0) - 1;
    inner = c_a.size(1);
    Bieq.set_size(c_a.size(0));
    for (c_i = 0; c_i <= mc; c_i++) {
      Bieq[c_i] = 0.0;
    }
    for (k = 0; k < inner; k++) {
      numRead = k * c_a.size(0);
      for (c_i = 0; c_i <= mc; c_i++) {
        Bieq[c_i] = Bieq[c_i] + c_a[numRead + c_i] * p_data[k];
      }
    }
    boffset = Bieq.size(0);
    for (i1 = 0; i1 < boffset; i1++) {
      Bieq[i1] = Bieq[i1] + Dieq[i1];
    }
    //  #The active-set QP algorithm requires that the Hessian matrix be
    //  positive definite. To determine whether H is positive definite, use the
    //  chol function.
    //      [L,~] = chol(obj.H,'lower');
    //      Linv = linsolve(L, eye(size(L)) ,struct('LT',true));
    mc = b_a.size(0) - 1;
    inner = b_a.size(1);
    Dieq.set_size(b_a.size(0));
    for (c_i = 0; c_i <= mc; c_i++) {
      Dieq[c_i] = 0.0;
    }
    for (k = 0; k < inner; k++) {
      numRead = k * b_a.size(0);
      for (c_i = 0; c_i <= mc; c_i++) {
        Dieq[c_i] = Dieq[c_i] + b_a[numRead + c_i] * p_data[k];
      }
    }
    Aeq.set_size(6, bytesOut + 8);
    for (i1 = 0; i1 < 4; i1++) {
      for (c = 0; c < 6; c++) {
        Aeq[c + 6 * i1] = AC.T_a[c + 6 * i1];
      }
    }
    Aeq[24] = -1.4142135623730949 * d_prv_Jcb_Tau_tmp * AC.F[0];
    Aeq[30] = -1.4142135623730949 * prv_Jcb_Tau_tmp * AC.F[1];
    Aeq[36] = -1.4142135623730949 * g_prv_Jcb_Tau_tmp * AC.F[2];
    Aeq[42] = -1.4142135623730949 * b_prv_Jcb_Tau_tmp * AC.F[3];
    Aeq[25] = -1.4142135623730949 * std::sin(prv_A_idx_0) * AC.F[0];
    Aeq[31] = 1.4142135623730949 * prv_Jcb_Tau_tmp * AC.F[1];
    Aeq[37] = -1.4142135623730949 * std::sin(prv_A_idx_2) * AC.F[2];
    Aeq[43] = 1.4142135623730949 * b_prv_Jcb_Tau_tmp * AC.F[3];
    Aeq[26] = -c_prv_Jcb_Tau_tmp * AC.F[0];
    Aeq[32] = -e_prv_Jcb_Tau_tmp * AC.F[1];
    Aeq[38] = -f_prv_Jcb_Tau_tmp * AC.F[2];
    Aeq[44] = -h_prv_Jcb_Tau_tmp * AC.F[3];
    Aeq[27] = prv_Ly * c_prv_Jcb_Tau_tmp * AC.F[0] +
              Lad * 1.4142135623730949 * d_prv_Jcb_Tau_tmp * AC.F[0];
    Aeq[33] = -prv_Ly * e_prv_Jcb_Tau_tmp * AC.F[1] -
              Lad * 1.4142135623730949 * prv_Jcb_Tau_tmp * AC.F[1];
    Aeq[39] = -prv_Ly * f_prv_Jcb_Tau_tmp * AC.F[2] +
              Lad * 1.4142135623730949 * g_prv_Jcb_Tau_tmp * AC.F[2];
    Aeq[45] = prv_Ly * h_prv_Jcb_Tau_tmp * AC.F[3] -
              Lad * 1.4142135623730949 * b_prv_Jcb_Tau_tmp * AC.F[3];
    Aeq[28] = prv_Lx * c_prv_Jcb_Tau_tmp * AC.F[0] +
              Lad * 1.4142135623730949 * std::sin(prv_A_idx_0) * AC.F[0];
    Aeq[34] = prv_Lx * e_prv_Jcb_Tau_tmp * AC.F[1] +
              Lad * 1.4142135623730949 * std::sin(prv_A_idx_1) * AC.F[1];
    Aeq[40] = -prv_Lx * f_prv_Jcb_Tau_tmp * AC.F[2] +
              Lad * 1.4142135623730949 * std::sin(prv_A_idx_2) * AC.F[2];
    Aeq[46] = -prv_Lx * h_prv_Jcb_Tau_tmp * AC.F[3] +
              Lad * 1.4142135623730949 * std::sin(prv_A_idx_3) * AC.F[3];
    prv_Lx += prv_Ly;
    prv_Ly = -prv_Lx * 1.4142135623730949;
    Aeq[29] = prv_Ly * d_prv_Jcb_Tau_tmp * AC.F[0] +
              Lad * 1.4142135623730949 * c_prv_Jcb_Tau_tmp * AC.F[0];
    prv_Lx *= 1.4142135623730949;
    Aeq[35] = prv_Lx * prv_Jcb_Tau_tmp * AC.F[1] -
              Lad * 1.4142135623730949 * e_prv_Jcb_Tau_tmp * AC.F[1];
    Aeq[41] = prv_Lx * g_prv_Jcb_Tau_tmp * AC.F[2] +
              Lad * 1.4142135623730949 * f_prv_Jcb_Tau_tmp * AC.F[2];
    Aeq[47] = prv_Ly * b_prv_Jcb_Tau_tmp * AC.F[3] -
              Lad * 1.4142135623730949 * h_prv_Jcb_Tau_tmp * AC.F[3];
    for (i1 = 0; i1 < bytesOut; i1++) {
      for (c = 0; c < 6; c++) {
        Aeq[c + 6 * (i1 + 8)] = b_I[c + 6 * i1];
      }
    }
    numRead = 6 * Aeq.size(1);
    a = true;
    for (k = 0; k < numRead; k++) {
      if ((!a) || (std::isinf(Aeq[k]) || std::isnan(Aeq[k]))) {
        a = false;
      }
    }
    if (a) {
      coder::internal::svd(Aeq, s_data, &numRead);
    } else {
      for (i1 = 0; i1 < 6; i1++) {
        s_data[i1] = rtNaN;
      }
    }
    prv_Lx = std::abs(s_data[0]);
    if ((!std::isinf(prv_Lx)) && (!std::isnan(prv_Lx)) &&
        (!(prv_Lx <= 2.2250738585072014E-308))) {
      frexp(prv_Lx, &exponent);
    }
    b_a.set_size(AC.Aieq.size(0), AC.Aieq.size(1));
    boffset = AC.Aieq.size(0) * AC.Aieq.size(1);
    for (i1 = 0; i1 < boffset; i1++) {
      b_a[i1] = -AC.Aieq[i1];
    }
    if ((b_a.size(0) != 0) && (b_a.size(1) != 0)) {
      sizes_idx_1 = b_a.size(1);
    } else {
      sizes_idx_1 = Aeq.size(1);
    }
    mc = AC.H.size(1);
    inner = AC.H.size(0);
    numRead = AC.H.size(1);
    c_a.set_size(AC.H.size(1), AC.H.size(1));
    for (c = 0; c < numRead; c++) {
      bytesOut = c * mc;
      boffset = c * AC.H.size(0);
      for (c_i = 0; c_i < mc; c_i++) {
        c_a[bytesOut + c_i] = 0.0;
      }
      for (k = 0; k < inner; k++) {
        prv_Lx = AC.H[boffset + k];
        for (c_i = 0; c_i < mc; c_i++) {
          i1 = bytesOut + c_i;
          c_a[i1] = c_a[i1] + AC.H[c_i * AC.H.size(0) + k] * prv_Lx;
        }
      }
    }
    iA1.set_size(Bieq.size(0) + 6);
    boffset = Bieq.size(0);
    for (i1 = 0; i1 < boffset; i1++) {
      iA1[i1] = 0;
    }
    for (i1 = 0; i1 < 6; i1++) {
      iA1[i1 + Bieq.size(0)] = 1;
    }
    if ((b_a.size(0) != 0) && (b_a.size(1) != 0)) {
      numRead = b_a.size(0);
    } else {
      numRead = 0;
    }
    if ((b_a.size(0) != 0) && (b_a.size(1) != 0)) {
      i1 = b_a.size(0);
    } else {
      i1 = 0;
    }
    d_a.set_size(i1 + 6, sizes_idx_1);
    for (i1 = 0; i1 < sizes_idx_1; i1++) {
      for (c = 0; c < numRead; c++) {
        d_a[c + d_a.size(0) * i1] = b_a[c + numRead * i1];
      }
    }
    for (i1 = 0; i1 < sizes_idx_1; i1++) {
      for (c = 0; c < 6; c++) {
        d_a[(c + numRead) + d_a.size(0) * i1] = Aeq[c + 6 * i1];
      }
    }
    b_Bieq.set_size(Bieq.size(0) + Dieq.size(0));
    boffset = Bieq.size(0);
    for (i1 = 0; i1 < boffset; i1++) {
      b_Bieq[i1] = -Bieq[i1];
    }
    boffset = Dieq.size(0);
    for (i1 = 0; i1 < boffset; i1++) {
      b_Bieq[i1 + Bieq.size(0)] = Dieq[i1];
    }
    prv_Lx = static_cast<double>(AC.Aieq.size(0)) + 6.0;
    if (prv_Lx < 32768.0) {
      if (prv_Lx >= -32768.0) {
        i3 = static_cast<short>(prv_Lx);
      } else {
        i3 = MIN_int16_T;
      }
    } else {
      i3 = MAX_int16_T;
    }
    i1 = AC.H.size(0);
    if (i1 > 32767) {
      i1 = 32767;
    } else if (i1 < -32768) {
      i1 = -32768;
    }
    b_a.set_size(AC.H.size(0), AC.H.size(1));
    boffset = AC.H.size(0) * AC.H.size(1) - 1;
    for (c = 0; c <= boffset; c++) {
      b_a[c] = AC.H[c];
    }
    Dieq.set_size(AC.f.size(0));
    boffset = AC.f.size(0) - 1;
    for (c = 0; c <= boffset; c++) {
      Dieq[c] = AC.f[c];
    }
    coder::qpkwik(b_a, c_a, Dieq, d_a, b_Bieq, iA1, i3, static_cast<short>(i1),
                  r, lam, &prv_Lx);
    boffset = r.size(0);
    AC.X.set_size(r.size(0));
    for (i1 = 0; i1 < boffset; i1++) {
      AC.X[i1] = r[i1];
    }
    AC.F[0] = AC.X[0];
    AC.F[1] = AC.X[1];
    AC.F[2] = AC.X[2];
    AC.F[3] = AC.X[3];
    AC.A[0] += AC.X[4];
    AC.A[1] += AC.X[5];
    AC.A[2] += AC.X[6];
    AC.A[3] += AC.X[7];
    numRead = memcmp(&AC.angles_out_unit[0], &b[0], 3);
    if (numRead == 0) {
      prv_A_idx_0 = AC.A[0] / 3.1415926535897931;
      prv_A_idx_1 = AC.A[1] / 3.1415926535897931;
      prv_A_idx_2 = AC.A[2] / 3.1415926535897931;
      prv_A_idx_3 = AC.A[3] / 3.1415926535897931;
      prv_A_idx_0 *= 180.0;
      prv_A_idx_1 *= 180.0;
      prv_A_idx_2 *= 180.0;
      prv_A_idx_3 *= 180.0;
    } else {
      numRead = memcmp(&AC.angles_out_unit[0], &b_b[0], 3);
      if (numRead == 0) {
        prv_A_idx_0 = AC.A[0];
        prv_A_idx_1 = AC.A[1];
        prv_A_idx_2 = AC.A[2];
        prv_A_idx_3 = AC.A[3];
      } else {
        //                  printf('Error angle ouput metric')
        prv_A_idx_0 = 0.0;
        prv_A_idx_1 = 0.0;
        prv_A_idx_2 = 0.0;
        prv_A_idx_3 = 0.0;
      }
    }
    //             %% ====================Calculate for the next
    //             loop===================
    AC.Update_Ta_Matrix(dv);
    std::copy(&dv[0], &dv[24], &AC.T_a[0]);
    //             %% ======================Output============================
    prv_Ly = AC.F[0];
    Force_Out_idx_1 = AC.F[1];
    t = AC.F[2];
    Lad = AC.F[3];
    //             %% OUPUT Force + Angle + Tau_Out + Tau_In +......
    //              [Outputs] =
    //              [Force_Out,Angle_Out,Tau_Out',Tau_in',norm(Tau_Out'-Tau_in')];
    for (i1 = 0; i1 < 6; i1++) {
      prv_Lx = AC.T_a[i1] * AC.F[0];
      prv_Lx += AC.T_a[i1 + 6] * AC.F[1];
      prv_Lx += AC.T_a[i1 + 12] * AC.F[2];
      prv_Lx += AC.T_a[i1 + 18] * AC.F[3];
      s_data[i1] = prv_Lx;
    }
    for (i1 = 0; i1 < 6; i1++) {
      Act_Out[i1] = s_data[i1];
    }
    // note output must be 1xn - row matrix;
    //      Tau_Out(:,k) = Act_Out;%double(subs(Taf))';
    //      Tau_In(:, k) = Tau_in(:,i)';
    //      Angle_Out(:, k) = Act_Out(7:10);
    //      Force_Out(:, k) = Act_Out(11:14);
    //      Time(k) = k*.01;
    coder::getfilestar(static_cast<double>(fileid), &filestar, &a);
    if (!(filestar == b_NULL)) {
      fprintf(filestar,
              "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f  %.3f %.3f "
              "%.3f %.3f ",
              Act_Out[0], Act_Out[1], Act_Out[2], Act_Out[3], Act_Out[4],
              Act_Out[5], prv_A_idx_0, prv_A_idx_1, prv_A_idx_2, prv_A_idx_3,
              prv_Ly, Force_Out_idx_1, t, Lad);
      if (a) {
        fflush(filestar);
      }
    }
    // toc
  }
  coder::cfclose(static_cast<double>(fileid));
}

//
// File trailer for Test_Actuator_Mod_V4.cpp
//
// [EOF]
//
