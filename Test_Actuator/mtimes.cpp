//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 2U> &B
//                ::coder::array<double, 1U> &C
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void mtimes(const ::coder::array<double, 2U> &A,
            const ::coder::array<double, 2U> &B, ::coder::array<double, 1U> &C)
{
  int i;
  int inner;
  int mc;
  mc = A.size(0) - 1;
  inner = A.size(1);
  C.set_size(A.size(0));
  for (i = 0; i <= mc; i++) {
    C[i] = 0.0;
  }
  for (int k{0}; k < inner; k++) {
    int aoffset;
    aoffset = k * A.size(0);
    for (i = 0; i <= mc; i++) {
      C[i] = C[i] + A[aoffset + i] * B[k];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &B
//                ::coder::array<double, 2U> &C
// Return Type  : void
//
void mtimes(const ::coder::array<double, 2U> &B, ::coder::array<double, 2U> &C)
{
  static const double dv[36]{0.75, 0.0,  0.0, 0.0,  0.0, 0.0,  0.0, 0.75, 0.0,
                             0.0,  0.0,  0.0, 0.0,  0.0, 0.75, 0.0, 0.0,  0.0,
                             0.0,  0.0,  0.0, 0.75, 0.0, 0.0,  0.0, 0.0,  0.0,
                             0.0,  0.75, 0.0, 0.0,  0.0, 0.0,  0.0, 0.0,  0.75};
  int n;
  n = B.size(1);
  C.set_size(6, B.size(1));
  for (int j{0}; j < n; j++) {
    int boffset;
    int coffset;
    coffset = j * 6;
    boffset = j * B.size(0);
    for (int i{0}; i < 6; i++) {
      double s;
      s = 0.0;
      for (int k{0}; k < 6; k++) {
        s += dv[k * 6 + i] * B[boffset + k];
      }
      C[coffset + i] = s;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for mtimes.cpp
//
// [EOF]
//
