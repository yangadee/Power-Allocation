//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgeqrf.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "xgeqrf.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
static double rt_hypotd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!std::isnan(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

//
// Arguments    : ::coder::array<double, 2U> &A
//                ::coder::array<double, 1U> &tau
// Return Type  : void
//
namespace coder {
namespace internal {
namespace lapack {
void xgeqrf(::coder::array<double, 2U> &A, ::coder::array<double, 1U> &tau)
{
  array<double, 1U> work;
  int i;
  int knt;
  int m;
  int minmana;
  int minmn;
  int n;
  m = A.size(0) - 1;
  n = A.size(1);
  knt = A.size(0);
  minmana = A.size(1);
  if (knt < minmana) {
    minmana = knt;
  }
  knt = A.size(0);
  minmn = A.size(1);
  if (knt < minmn) {
    minmn = knt;
  }
  tau.set_size(minmana);
  for (i = 0; i < minmana; i++) {
    tau[i] = 0.0;
  }
  if ((A.size(0) != 0) && (A.size(1) != 0) && (minmn >= 1)) {
    int lda;
    lda = A.size(0);
    work.set_size(A.size(1));
    knt = A.size(1);
    for (i = 0; i < knt; i++) {
      work[i] = 0.0;
    }
    for (int b_i{0}; b_i < minmn; b_i++) {
      double atmp;
      double c;
      int ii;
      int k;
      int mmi_tmp;
      ii = b_i * lda + b_i;
      mmi_tmp = m - b_i;
      if (b_i + 1 < m + 1) {
        atmp = A[ii];
        minmana = ii + 2;
        tau[b_i] = 0.0;
        if (mmi_tmp + 1 > 0) {
          c = blas::b_xnrm2(mmi_tmp, A, ii + 2);
          if (c != 0.0) {
            double beta1;
            beta1 = rt_hypotd_snf(A[ii], c);
            if (A[ii] >= 0.0) {
              beta1 = -beta1;
            }
            if (std::abs(beta1) < 1.0020841800044864E-292) {
              knt = -1;
              i = (ii + mmi_tmp) + 1;
              do {
                knt++;
                for (k = minmana; k <= i; k++) {
                  A[k - 1] = 9.9792015476736E+291 * A[k - 1];
                }
                beta1 *= 9.9792015476736E+291;
                atmp *= 9.9792015476736E+291;
              } while (!(std::abs(beta1) >= 1.0020841800044864E-292));
              beta1 = rt_hypotd_snf(atmp, blas::b_xnrm2(mmi_tmp, A, ii + 2));
              if (atmp >= 0.0) {
                beta1 = -beta1;
              }
              tau[b_i] = (beta1 - atmp) / beta1;
              c = 1.0 / (atmp - beta1);
              for (k = minmana; k <= i; k++) {
                A[k - 1] = c * A[k - 1];
              }
              for (k = 0; k <= knt; k++) {
                beta1 *= 1.0020841800044864E-292;
              }
              atmp = beta1;
            } else {
              tau[b_i] = (beta1 - A[ii]) / beta1;
              c = 1.0 / (A[ii] - beta1);
              i = (ii + mmi_tmp) + 1;
              for (k = minmana; k <= i; k++) {
                A[k - 1] = c * A[k - 1];
              }
              atmp = beta1;
            }
          }
        }
        A[ii] = atmp;
      } else {
        tau[b_i] = 0.0;
      }
      if (b_i + 1 < n) {
        int ia;
        int ic0;
        int lastv;
        atmp = A[ii];
        A[ii] = 1.0;
        ic0 = (ii + lda) + 1;
        if (tau[b_i] != 0.0) {
          boolean_T exitg2;
          lastv = mmi_tmp + 1;
          knt = ii + mmi_tmp;
          while ((lastv > 0) && (A[knt] == 0.0)) {
            lastv--;
            knt--;
          }
          knt = (n - b_i) - 1;
          exitg2 = false;
          while ((!exitg2) && (knt > 0)) {
            int exitg1;
            minmana = ic0 + (knt - 1) * lda;
            ia = minmana;
            do {
              exitg1 = 0;
              if (ia <= (minmana + lastv) - 1) {
                if (A[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                knt--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          knt = 0;
        }
        if (lastv > 0) {
          if (knt != 0) {
            for (mmi_tmp = 0; mmi_tmp < knt; mmi_tmp++) {
              work[mmi_tmp] = 0.0;
            }
            mmi_tmp = 0;
            i = ic0 + lda * (knt - 1);
            for (k = ic0; lda < 0 ? k >= i : k <= i; k += lda) {
              c = 0.0;
              minmana = (k + lastv) - 1;
              for (ia = k; ia <= minmana; ia++) {
                c += A[ia - 1] * A[(ii + ia) - k];
              }
              work[mmi_tmp] = work[mmi_tmp] + c;
              mmi_tmp++;
            }
          }
          blas::xgerc(lastv, knt, -tau[b_i], ii + 1, work, A, ic0, lda);
        }
        A[ii] = atmp;
      }
    }
  }
}

} // namespace lapack
} // namespace internal
} // namespace coder

//
// File trailer for xgeqrf.cpp
//
// [EOF]
//
