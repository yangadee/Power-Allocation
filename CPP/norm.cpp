//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "norm.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[14]
// Return Type  : double
//
namespace coder
{
  double b_norm(const double x[14])
  {
    double scale;
    double y;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    for (int k = 0; k < 14; k++) {
      double absxk;
      absxk = std::abs(x[k]);
      if (absxk > scale) {
        double t;
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        double t;
        t = absxk / scale;
        y += t * t;
      }
    }

    return scale * std::sqrt(y);
  }
}

//
// File trailer for norm.cpp
//
// [EOF]
//
