//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Tau2Force.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//
#ifndef TAU2FORCE_H
#define TAU2FORCE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <stdio.h>

// Type Definitions
struct struct_T
{
  double tv_sec;
  double tv_nsec;
};

struct Fun_Local_Convex_QP_4CppPersistentData
{
  FILE * eml_openfiles[20];
  bool eml_autoflush[20];
  struct_T savedTime;
  bool savedTime_not_empty;
};

struct Fun_Local_Convex_QP_4CppStackData
{
  Fun_Local_Convex_QP_4CppPersistentData *pd;
};

class Tau2Force
{
 public:
  Tau2Force();
  ~Tau2Force();
  void Fun_Local_Convex_QP_4Cpp();
  Fun_Local_Convex_QP_4CppStackData *getStackData();
 private:
  Fun_Local_Convex_QP_4CppPersistentData pd_;
  Fun_Local_Convex_QP_4CppStackData SD_;
};

#endif

//
// File trailer for Tau2Force.h
//
// [EOF]
//
