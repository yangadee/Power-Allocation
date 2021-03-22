//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const double x[14]
// Return Type  : double
//
namespace coder
{
  namespace internal
  {
    double maximum(const double x[14])
    {
      double ex;
      int idx;
      int k;
      if (!std::isnan(x[0])) {
        idx = 1;
      } else {
        bool exitg1;
        idx = 0;
        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k < 15)) {
          if (!std::isnan(x[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx == 0) {
        ex = x[0];
      } else {
        ex = x[idx - 1];
        idx++;
        for (k = idx; k < 15; k++) {
          double d;
          d = x[k - 1];
          if (ex < d) {
            ex = d;
          }
        }
      }

      return ex;
    }
  }
}

//
// File trailer for minOrMax.cpp
//
// [EOF]
//
