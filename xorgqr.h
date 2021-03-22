//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xorgqr.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

#ifndef XORGQR_H
#define XORGQR_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace lapack {
void xorgqr(int m, int n, int k, ::coder::array<double, 2U> &A, int lda,
            const ::coder::array<double, 1U> &tau);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xorgqr.h
//
// [EOF]
//
