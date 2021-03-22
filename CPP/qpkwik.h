//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpkwik.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//
#ifndef QPKWIK_H
#define QPKWIK_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  void qpkwik(const double Linv[196], const double Hinv[196], const double f[14],
              const double Ac[420], const double b[30], short iA[30], double x
              [14], double lambda[30], double *status);
}

#endif

//
// File trailer for qpkwik.h
//
// [EOF]
//
