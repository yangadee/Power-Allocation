//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "eye.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : double varargin_1
//                ::coder::array<double, 2U> &b_I
// Return Type  : void
//
namespace coder {
void eye(double varargin_1, ::coder::array<double, 2U> &b_I)
{
  double t;
  int loop_ub;
  int m;
  if (varargin_1 < 0.0) {
    t = 0.0;
  } else {
    t = varargin_1;
  }
  m = static_cast<int>(t);
  b_I.set_size(static_cast<int>(t), static_cast<int>(t));
  loop_ub = static_cast<int>(t) * static_cast<int>(t);
  for (int i{0}; i < loop_ub; i++) {
    b_I[i] = 0.0;
  }
  if (static_cast<int>(t) > 0) {
    for (loop_ub = 0; loop_ub < m; loop_ub++) {
      b_I[loop_ub + b_I.size(0) * loop_ub] = 1.0;
    }
  }
}

} // namespace coder

//
// File trailer for eye.cpp
//
// [EOF]
//
