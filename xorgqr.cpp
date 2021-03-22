//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xorgqr.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "xorgqr.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : int m
//                int n
//                int k
//                ::coder::array<double, 2U> &A
//                int lda
//                const ::coder::array<double, 1U> &tau
// Return Type  : void
//
namespace coder {
namespace internal {
namespace lapack {
void xorgqr(int m, int n, int k, ::coder::array<double, 2U> &A, int lda,
            const ::coder::array<double, 1U> &tau)
{
  array<double, 1U> work;
  if (n >= 1) {
    int b_i;
    int b_k;
    int c_i;
    int i;
    int ia;
    int itau;
    i = n - 1;
    for (b_i = k; b_i <= i; b_i++) {
      ia = b_i * lda;
      b_k = m - 1;
      for (c_i = 0; c_i <= b_k; c_i++) {
        A[ia + c_i] = 0.0;
      }
      A[ia + b_i] = 1.0;
    }
    itau = k - 1;
    b_i = A.size(1);
    work.set_size(b_i);
    for (i = 0; i < b_i; i++) {
      work[i] = 0.0;
    }
    for (c_i = k; c_i >= 1; c_i--) {
      int iaii;
      iaii = c_i + (c_i - 1) * lda;
      if (c_i < n) {
        int ic0;
        int lastc;
        int lastv;
        A[iaii - 1] = 1.0;
        ic0 = iaii + lda;
        if (tau[itau] != 0.0) {
          boolean_T exitg2;
          lastv = (m - c_i) + 1;
          b_i = (iaii + m) - c_i;
          while ((lastv > 0) && (A[b_i - 1] == 0.0)) {
            lastv--;
            b_i--;
          }
          lastc = n - c_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            int exitg1;
            b_i = ic0 + (lastc - 1) * lda;
            ia = b_i;
            do {
              exitg1 = 0;
              if (ia <= (b_i + lastv) - 1) {
                if (A[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }
        if (lastv > 0) {
          if (lastc != 0) {
            for (b_i = 0; b_i < lastc; b_i++) {
              work[b_i] = 0.0;
            }
            b_i = 0;
            i = ic0 + lda * (lastc - 1);
            for (int iac{ic0}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
              double c;
              c = 0.0;
              b_k = (iac + lastv) - 1;
              for (ia = iac; ia <= b_k; ia++) {
                c += A[ia - 1] * A[((iaii + ia) - iac) - 1];
              }
              work[b_i] = work[b_i] + c;
              b_i++;
            }
          }
          blas::xgerc(lastv, lastc, -tau[itau], iaii, work, A, ic0, lda);
        }
      }
      if (c_i < m) {
        b_i = iaii + 1;
        i = (iaii + m) - c_i;
        for (b_k = b_i; b_k <= i; b_k++) {
          A[b_k - 1] = -tau[itau] * A[b_k - 1];
        }
      }
      A[iaii - 1] = 1.0 - tau[itau];
      for (b_i = 0; b_i <= c_i - 2; b_i++) {
        A[(iaii - b_i) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace lapack
} // namespace internal
} // namespace coder

//
// File trailer for xorgqr.cpp
//
// [EOF]
//
