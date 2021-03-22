//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgerc.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//
#ifndef XGERC_H
#define XGERC_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace blas
    {
      void xgerc(int m, int n, double alpha1, int ix0, const double y[14],
                 double A[196], int ia0);
    }
  }
}

#endif

//
// File trailer for xgerc.h
//
// [EOF]
//
