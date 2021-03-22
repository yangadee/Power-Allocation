//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Tau2Force.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "Tau2Force.h"
#include "fileManager.h"
#include "qpkwik.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "tic.h"
#include "toc.h"
#include "coder_array.h"
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

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
//            Tau_in is control force by guidance law in matrix format with matrix size = nx1
//            T_ext is extended thrust configuration matrix which define by user
//
//
//  QP form: J= x'Hx + f'x
//                        Aieq*x <= Bieq
//                        Aeq*x   = Beq
//  in this function x = combination of Force_out and slack variable s
//                                      where s has the same size of Tau_in
//  function:      [x,exitflag,iA,lambda] = mpcActiveSetSolver(H,f,Aieq,bieq,Aeq,beq,iA0,options)
// Arguments    : void
// Return Type  : void
//
void Tau2Force::Fun_Local_Convex_QP_4Cpp()
{
  static const double dv5[196]{
    10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      1.0
  };

  static const double dv6[196]{
    100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0
  };

  static const double dv[8]{
    3.1415926535897931, 3.1415926535897931, 3.1415926535897931,
      3.1415926535897931, -3.1415926535897931, -3.1415926535897931,
      -3.1415926535897931, -3.1415926535897931
  };

  static const double dv1[8]{
    0.017453292519943295, 0.017453292519943295, 0.017453292519943295,
      0.017453292519943295, -0.017453292519943295, -0.017453292519943295,
      -0.017453292519943295, -0.017453292519943295
  };

  static const signed char c_a[720]{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1
  };

  static const signed char iv3[336]{
    -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
  };

  static const signed char d_a[180]{
    1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0
  };

  static const signed char iv1[36]{
    -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0,
      0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1
  };

  static const signed char iv2[30]{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1,
      1, 1, 1, 1
  };

  static const signed char iv[8]{
    50, 50, 50, 50, -50, -50, -50, -50
  };

  FILE * b_NULL;
  FILE * filestar;
  size_t nBytes;
  coder::array<double, 2U> At;
  coder::array<double, 2U> Tau_in;
  coder::array<double, 1U> b_At;
  double tbuf[1024];
  double dv4[420];
  double Aeq[84];
  double lam[30];
  double p_data[30];
  double T_a[24];
  double b_a[24];
  double dv2[24];
  double dv3[14];
  double x[14];
  double s[6];
  double T_a_tmp;
  double a1;
  double a2;
  double a3;
  double a4;
  double absx;
  double f1;
  double f2;
  double f3;
  double f4;
  int bytesOut;
  int c;
  int exponent;
  int i;
  int loop_ub;
  int numRead;
  short iA1[30];
  signed char fileid;
  bool a;

  //  QP crition: J = f'Wf + (f-f0)'M(f-f0)+ s'Qs
  //  x = [f, da, s]
  //  load Tau.mat                  %±±¨î¤O (Utau)
  // %% Load bin file
  //  % %To Save bin file
  //  FID = fopen(FileName, 'w');
  //  fwrite(FID, Utau, 'double');
  //  fclose(FID);
  //  % %To load Save bin file
  fileid = coder::cfopen(this, "Utau3.dat", "rb");
  nBytes = sizeof(double);
  coder::getfilestar(this, static_cast<double>(fileid), &filestar, &a);
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
        loop_ub = 0;
      } else {
        loop_ub = c;
      }

      b_At.set_size((At.size(0) + loop_ub));
      numRead = At.size(0);
      for (i = 0; i < numRead; i++) {
        b_At[i] = At[i];
      }

      for (i = 0; i < loop_ub; i++) {
        b_At[i + At.size(0)] = tbuf[i];
      }

      At.set_size(b_At.size(0), 1);
      loop_ub = b_At.size(0);
      for (i = 0; i < loop_ub; i++) {
        At[i] = b_At[i];
      }

      bytesOut += c;
    }
  }

  if (At.size(0) >= 6) {
    Tau_in.set_size(6, div_nde_s32_ceiling(bytesOut + 1, 6));
    for (numRead = 0; numRead <= bytesOut; numRead++) {
      Tau_in[numRead] = At[numRead];
    }

    i = bytesOut + 2;
    c = 6 * Tau_in.size(1);
    for (numRead = i; numRead <= c; numRead++) {
      Tau_in[numRead - 1] = 0.0;
    }
  } else {
    Tau_in.set_size(At.size(0), 1);
    loop_ub = At.size(0) * At.size(1);
    for (i = 0; i < loop_ub; i++) {
      Tau_in[i] = At[i];
    }
  }

  coder::cfclose(this, static_cast<double>(fileid));

  //  for results Output
  fileid = coder::cfopen(this, "ResultOut1.txt", "wb");
  coder::cfclose(this, static_cast<double>(fileid));
  fileid = coder::cfopen(this, "ResultOut1.txt", "ab");

  // size(T_a,2);        % length of solved f (equal to number of actuator, r = 4 in case of quadrotor) 
  // size(Tau_in',2); % length of Tau_in (equal to number of DOF,n = 6 in case of full DOF control) 
  a1 = 0.0;
  a2 = 0.0;
  a3 = 0.0;
  a4 = 0.0;
  f1 = 0.0;
  f2 = 0.0;
  f3 = 0.0;
  f4 = 0.0;

  //  in rad/s
  //  inequality constrains
  // 6r x (n+3r)
  // 6r x (n+6r)
  //  f % r x r = 4x4
  //  f % r x r = 12x12
  //  Tau % equal to size of Tau_in = nxn = 6x6
  loop_ub = Tau_in.size(0);
  b_NULL = NULL;
  for (bytesOut = 0; bytesOut < 2; bytesOut++) {
    double absxk;
    double b_T_a_tmp;
    double c_T_a_tmp;
    double d_T_a_tmp;
    double scale;
    double t;
    double unnamed_idx_0;
    double unnamed_idx_1;
    double unnamed_idx_2;
    double unnamed_idx_3;

    // size(Tau_in,2)
    coder::tic(this);
    for (i = 0; i < loop_ub; i++) {
      p_data[i] = Tau_in[i + Tau_in.size(0) * bytesOut];
    }

    for (i = 0; i < 8; i++) {
      numRead = i + loop_ub;
      p_data[numRead] = iv[i];
      p_data[numRead + 8] = dv[i];
      p_data[numRead + 16] = dv1[i];
    }

    // Hessian matrix combined P, Q and R
    //  note: this is not control forces f, it is  linear multiplier vector, f = 0 matrix in this case 
    absx = std::cos(a2);
    T_a_tmp = std::cos(a4);
    scale = std::sin(a1);
    absxk = std::sin(a2);
    t = std::sin(a3);
    b_T_a_tmp = std::sin(a4);
    c_T_a_tmp = std::cos(a1);
    d_T_a_tmp = std::cos(a3);
    T_a[0] = 0.707 * c_T_a_tmp;
    T_a[6] = 0.707 * absx;
    T_a[12] = 0.707 * d_T_a_tmp;
    T_a[18] = 0.707 * T_a_tmp;
    T_a[1] = 0.707 * std::cos(a1);
    T_a[7] = -0.707 * absx;
    T_a[13] = 0.707 * std::cos(a3);
    T_a[19] = -0.707 * T_a_tmp;
    T_a[2] = -scale;
    T_a[8] = -absxk;
    T_a[14] = -t;
    T_a[20] = -b_T_a_tmp;
    T_a[3] = 100.0 * scale;
    T_a[9] = -100.0 * absxk;
    T_a[15] = -100.0 * t;
    T_a[21] = 100.0 * b_T_a_tmp;
    T_a[4] = 100.0 * std::sin(a1);
    T_a[10] = 100.0 * absxk;
    T_a[16] = -100.0 * std::sin(a3);
    T_a[22] = -100.0 * b_T_a_tmp;
    T_a[5] = 141.4 * c_T_a_tmp;
    T_a[11] = -141.4 * absx;
    T_a[17] = -141.4 * d_T_a_tmp;
    T_a[23] = 141.4 * T_a_tmp;

    //  T_extended
    unnamed_idx_0 = f1;
    unnamed_idx_1 = f2;
    unnamed_idx_2 = f3;
    unnamed_idx_3 = f4;

    //  equality constrains
    for (i = 0; i < 4; i++) {
      for (c = 0; c < 6; c++) {
        numRead = c + 6 * i;
        Aeq[numRead] = T_a[numRead];
      }
    }

    Aeq[24] = -0.707 * scale * f1;
    Aeq[30] = -0.707 * absxk * f2;
    Aeq[36] = -0.707 * t * f3;
    Aeq[42] = -0.707 * b_T_a_tmp * f4;
    Aeq[25] = -0.707 * std::sin(a1) * f1;
    Aeq[31] = 0.707 * absxk * f2;
    Aeq[37] = -0.707 * std::sin(a3) * f3;
    Aeq[43] = 0.707 * b_T_a_tmp * f4;
    Aeq[26] = -c_T_a_tmp * f1;
    Aeq[32] = -absx * f2;
    Aeq[38] = -d_T_a_tmp * f3;
    Aeq[44] = -T_a_tmp * f4;
    Aeq[27] = 100.0 * c_T_a_tmp * f1;
    Aeq[33] = -100.0 * absx * f2;
    Aeq[39] = -100.0 * d_T_a_tmp * f3;
    Aeq[45] = 100.0 * T_a_tmp * f4;
    Aeq[28] = 100.0 * std::cos(a1) * f1;
    Aeq[34] = 100.0 * absx * f2;
    Aeq[40] = -100.0 * std::cos(a3) * f3;
    Aeq[46] = -100.0 * T_a_tmp * f4;
    Aeq[29] = -141.4 * scale * f1;
    Aeq[35] = 141.4 * absxk * f2;
    Aeq[41] = 141.4 * t * f3;
    Aeq[47] = -141.4 * b_T_a_tmp * f4;
    for (i = 0; i < 6; i++) {
      for (c = 0; c < 6; c++) {
        Aeq[c + 6 * (i + 8)] = iv1[c + 6 * i];
      }
    }

    //  inequality constrains
    //      Aieq = [   eye(r)   zeros(r,n+r);
    //                      -eye(r)   zeros(r,n+r);
    //                      zeros(r,r)  eye(r)  zeros(r,n);
    //                      zeros(r,r) -eye(r)  zeros(r,n);
    //                      zeros(r,r)  eye(r)  zeros(r,n);
    //                      zeros(r,r) -eye(r)  zeros(r,n);] ;%6r x (n+3r)
    //
    //      Cieq = [ zeros(r,n) eye(r)   zeros(r,5*r);
    //                      zeros(r,n+r)     -eye(r)   zeros(r,4*r);
    //                      zeros(r,n+2*r)  eye(r)   zeros(r,3*r);
    //                      zeros(r,n+3*r) -eye(r)   zeros(r,2*r);
    //                      zeros(r,n+4*r)   eye(r)   zeros(r,r);
    //                      zeros(r,n+5*r)  -eye(r); ]; %6r x (n+6r)
    // %%%%declear in header
    //  #The active-set QP algorithm requires that the Hessian matrix be positive definite. To determine whether H is positive definite, use the chol function. 
    //      [L,~] = chol(H,'lower');
    //      Linv = linsolve(L, eye(size(L)) ,struct('LT',true));
    a = true;
    for (numRead = 0; numRead < 84; numRead++) {
      if ((!a) || (std::isinf(Aeq[numRead]) || std::isnan(Aeq[numRead]))) {
        a = false;
      }
    }

    if (a) {
      coder::internal::svd(Aeq, s);
    } else {
      for (numRead = 0; numRead < 6; numRead++) {
        s[numRead] = rtNaN;
      }
    }

    absx = std::abs(s[0]);
    if ((!std::isinf(absx)) && (!std::isnan(absx)) && (!(absx <=
          2.2250738585072014E-308))) {
      frexp(absx, &exponent);
    }

    for (numRead = 0; numRead < 30; numRead++) {
      iA1[numRead] = iv2[numRead];
    }

    for (i = 0; i < 24; i++) {
      absx = 0.0;
      for (c = 0; c < 30; c++) {
        absx += static_cast<double>(c_a[i + 24 * c]) * p_data[c];
      }

      b_a[i] = absx;
    }

    dv2[8] = -a1;
    dv2[9] = -a2;
    dv2[10] = -a3;
    dv2[11] = -a4;
    dv2[12] = a1;
    dv2[13] = a2;
    dv2[14] = a3;
    dv2[15] = a4;
    for (i = 0; i < 8; i++) {
      dv2[i] = 0.0;
      dv2[i + 16] = 0.0;
    }

    for (i = 0; i < 6; i++) {
      absx = 0.0;
      for (c = 0; c < 30; c++) {
        absx += static_cast<double>(d_a[i + 6 * c]) * p_data[c];
      }

      s[i] = absx;
    }

    for (i = 0; i < 14; i++) {
      dv3[i] = 0.0;
      for (c = 0; c < 24; c++) {
        dv4[c + 30 * i] = iv3[c + 24 * i];
      }

      for (c = 0; c < 6; c++) {
        dv4[(c + 30 * i) + 24] = Aeq[c + 6 * i];
      }
    }

    for (i = 0; i < 24; i++) {
      p_data[i] = -(b_a[i] + dv2[i]);
    }

    for (i = 0; i < 6; i++) {
      p_data[i + 24] = s[i];
    }

    coder::qpkwik(dv5, dv6, dv3, dv4, p_data, iA1, x, lam, &absx);
    f1 = x[0];
    f2 = x[1];
    f3 = x[2];
    f4 = x[3];
    a1 += x[4];
    a2 += x[5];
    a3 += x[6];
    a4 += x[7];
    coder::toc(this);
    T_a_tmp = 0.0;
    scale = 3.3121686421112381E-170;
    for (numRead = 0; numRead < 6; numRead++) {
      absxk = std::abs(Tau_in[numRead + Tau_in.size(0) * bytesOut] -
                       (((T_a[numRead] * unnamed_idx_0 + T_a[numRead + 6] *
                          unnamed_idx_1) + T_a[numRead + 12] * unnamed_idx_2) +
                        T_a[numRead + 18] * unnamed_idx_3));
      if (absxk > scale) {
        t = scale / absxk;
        T_a_tmp = T_a_tmp * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        T_a_tmp += t * t;
      }
    }

    coder::getfilestar(this, static_cast<double>(fileid), &filestar, &a);
    if (!(filestar == b_NULL)) {
      fprintf(filestar, "%.0f %.3f %.3f %.3f %.3f  %.3f %.3f %.3f %.3f %.3f \n",
              absx, x[0], x[1], x[2], x[3], a1 / 3.1415926535897931 * 180.0, a2 /
              3.1415926535897931 * 180.0, a3 / 3.1415926535897931 * 180.0, a4 /
              3.1415926535897931 * 180.0, scale * std::sqrt(T_a_tmp));
      if (a) {
        fflush(filestar);
      }
    }
  }

  coder::cfclose(this, static_cast<double>(fileid));
}

//
// Arguments    : void
// Return Type  : void
//
Tau2Force::Tau2Force()
{
  FILE * a;
  this->SD_.pd = &this->pd_;
  this->pd_.savedTime_not_empty = false;
  a = NULL;
  for (int i = 0; i < 20; i++) {
    this->pd_.eml_autoflush[i] = false;
    this->pd_.eml_openfiles[i] = a;
  }
}

//
// Arguments    : void
// Return Type  : void
//
Tau2Force::~Tau2Force()
{
  // (no terminate code required)
}

//
// Arguments    : void
// Return Type  : Fun_Local_Convex_QP_4CppStackData *
//
Fun_Local_Convex_QP_4CppStackData *Tau2Force::getStackData()
{
  return &this->SD_;
}

//
// File trailer for Tau2Force.cpp
//
// [EOF]
//
