//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: norm.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "norm.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 1U> &x
// Return Type  : double
//
namespace coder {
double b_norm(const ::coder::array<double, 1U> &x)
{
  double scale;
  double y;
  int kend;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  kend = x.size(0);
  for (int k{0}; k < kend; k++) {
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

} // namespace coder

//
// File trailer for norm.cpp
//
// [EOF]
//
