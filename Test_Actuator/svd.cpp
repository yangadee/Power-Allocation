//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xrotg.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                double U_data[]
//                int *U_size
// Return Type  : void
//
namespace coder {
namespace internal {
void svd(const ::coder::array<double, 2U> &A, double U_data[], int *U_size)
{
  array<double, 2U> b_A;
  array<double, 1U> e;
  double s_data[7];
  double work[6];
  double nrm;
  double rt;
  double sm;
  double snorm;
  double sqds;
  double ztest;
  int b_i;
  int i;
  int iter;
  int k;
  int p;
  int q;
  int qjj;
  int qp1;
  int qs;
  b_A.set_size(6, A.size(1));
  i = 6 * A.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    b_A[b_i] = A[b_i];
  }
  p = A.size(1);
  for (b_i = 0; b_i < 7; b_i++) {
    s_data[b_i] = 0.0;
  }
  e.set_size(A.size(1));
  i = A.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    e[b_i] = 0.0;
  }
  for (i = 0; i < 6; i++) {
    work[i] = 0.0;
  }
  for (q = 0; q < 6; q++) {
    boolean_T apply_transform;
    qp1 = q + 2;
    i = (q + 6 * q) + 1;
    qs = 5 - q;
    apply_transform = false;
    if (q + 1 <= 5) {
      nrm = blas::xnrm2(6 - q, b_A, i);
      if (nrm > 0.0) {
        apply_transform = true;
        if (b_A[i - 1] < 0.0) {
          ztest = -nrm;
          s_data[q] = -nrm;
        } else {
          ztest = nrm;
          s_data[q] = nrm;
        }
        if (std::abs(ztest) >= 1.0020841800044864E-292) {
          nrm = 1.0 / ztest;
          b_i = (i - q) + 5;
          for (k = i; k <= b_i; k++) {
            b_A[k - 1] = nrm * b_A[k - 1];
          }
        } else {
          b_i = (i - q) + 5;
          for (k = i; k <= b_i; k++) {
            b_A[k - 1] = b_A[k - 1] / s_data[q];
          }
        }
        b_A[i - 1] = b_A[i - 1] + 1.0;
        s_data[q] = -s_data[q];
      } else {
        s_data[q] = 0.0;
      }
    }
    for (iter = qp1; iter <= p; iter++) {
      qjj = q + 6 * (iter - 1);
      if (apply_transform) {
        nrm = 0.0;
        for (k = 0; k <= qs; k++) {
          nrm += b_A[(i + k) - 1] * b_A[qjj + k];
        }
        nrm = -(nrm / b_A[q + 6 * q]);
        if (!(nrm == 0.0)) {
          for (k = 0; k <= qs; k++) {
            b_i = qjj + k;
            b_A[b_i] = b_A[b_i] + nrm * b_A[(i + k) - 1];
          }
        }
      }
      e[iter - 1] = b_A[qjj];
    }
    i = (p - q) - 1;
    nrm = blas::xnrm2(i, e, q + 2);
    if (nrm == 0.0) {
      e[q] = 0.0;
    } else {
      if (e[q + 1] < 0.0) {
        e[q] = -nrm;
      } else {
        e[q] = nrm;
      }
      nrm = e[q];
      if (std::abs(e[q]) >= 1.0020841800044864E-292) {
        nrm = 1.0 / e[q];
        b_i = (q + i) + 1;
        for (k = qp1; k <= b_i; k++) {
          e[k - 1] = nrm * e[k - 1];
        }
      } else {
        b_i = (q + i) + 1;
        for (k = qp1; k <= b_i; k++) {
          e[k - 1] = e[k - 1] / nrm;
        }
      }
      e[q + 1] = e[q + 1] + 1.0;
      e[q] = -e[q];
      if (q + 2 <= 6) {
        for (qjj = qp1; qjj < 7; qjj++) {
          work[qjj - 1] = 0.0;
        }
        for (iter = qp1; iter <= p; iter++) {
          if (5 - q >= 1) {
            ztest = e[iter - 1];
            if (!(ztest == 0.0)) {
              i = (q + 6 * (iter - 1)) + 1;
              b_i = 4 - q;
              for (k = 0; k <= b_i; k++) {
                qs = (q + k) + 1;
                work[qs] += ztest * b_A[i + k];
              }
            }
          }
        }
        for (iter = qp1; iter <= p; iter++) {
          nrm = -e[iter - 1] / e[q + 1];
          if ((5 - q >= 1) && (!(nrm == 0.0))) {
            i = (q + 6 * (iter - 1)) + 1;
            b_i = 4 - q;
            for (k = 0; k <= b_i; k++) {
              qs = i + k;
              b_A[qs] = b_A[qs] + nrm * work[(q + k) + 1];
            }
          }
        }
      }
    }
  }
  p = 5;
  s_data[5] = b_A[35];
  s_data[6] = 0.0;
  e[6] = 0.0;
  iter = 0;
  snorm = 0.0;
  for (q = 0; q < 7; q++) {
    ztest = s_data[q];
    sm = ztest;
    if (ztest != 0.0) {
      rt = std::abs(ztest);
      sm = rt;
      s_data[q] = rt;
      if (q + 1 < 7) {
        e[q] = e[q] / (ztest / rt);
      }
    }
    if ((q + 1 < 7) && (e[q] != 0.0)) {
      rt = std::abs(e[q]);
      nrm = rt / e[q];
      e[q] = rt;
      s_data[q + 1] *= nrm;
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(sm), std::abs(e[q])));
  }
  while ((p + 2 > 0) && (iter < 75)) {
    qjj = p;
    int exitg1;
    do {
      exitg1 = 0;
      q = qjj + 1;
      if (qjj + 1 == 0) {
        exitg1 = 1;
      } else {
        nrm = std::abs(e[qjj]);
        if ((nrm <= 2.2204460492503131E-16 *
                        (std::abs(s_data[qjj]) + std::abs(s_data[qjj + 1]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[qjj] = 0.0;
          exitg1 = 1;
        } else {
          qjj--;
        }
      }
    } while (exitg1 == 0);
    if (qjj + 1 == p + 1) {
      i = 4;
    } else {
      boolean_T exitg2;
      qs = p + 2;
      i = p + 2;
      exitg2 = false;
      while ((!exitg2) && (i >= qjj + 1)) {
        qs = i;
        if (i == qjj + 1) {
          exitg2 = true;
        } else {
          nrm = 0.0;
          if (i < p + 2) {
            nrm = std::abs(e[i - 1]);
          }
          if (i > qjj + 2) {
            nrm += std::abs(e[i - 2]);
          }
          ztest = std::abs(s_data[i - 1]);
          if ((ztest <= 2.2204460492503131E-16 * nrm) ||
              (ztest <= 1.0020841800044864E-292)) {
            s_data[i - 1] = 0.0;
            exitg2 = true;
          } else {
            i--;
          }
        }
      }
      if (qs == qjj + 1) {
        i = 3;
      } else if (qs == p + 2) {
        i = 1;
      } else {
        i = 2;
        q = qs;
      }
    }
    switch (i) {
    case 1:
      ztest = e[p];
      e[p] = 0.0;
      b_i = p + 1;
      for (k = b_i; k >= q + 1; k--) {
        blas::xrotg(&s_data[k - 1], &ztest, &rt, &sqds);
        if (k > q + 1) {
          sm = e[k - 2];
          ztest = -sqds * sm;
          e[k - 2] = sm * rt;
        }
      }
      break;
    case 2:
      ztest = e[q - 1];
      e[q - 1] = 0.0;
      for (k = q + 1; k <= p + 2; k++) {
        blas::xrotg(&s_data[k - 1], &ztest, &rt, &sqds);
        sm = e[k - 1];
        ztest = -sqds * sm;
        e[k - 1] = sm * rt;
      }
      break;
    case 3: {
      double scale;
      i = p + 1;
      nrm = s_data[p + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(s_data[p])),
                              std::abs(e[p])),
                    std::abs(s_data[q])),
          std::abs(e[q]));
      sm = nrm / scale;
      nrm = s_data[p] / scale;
      ztest = e[p] / scale;
      sqds = s_data[q] / scale;
      rt = ((nrm + sm) * (nrm - sm) + ztest * ztest) / 2.0;
      nrm = sm * ztest;
      nrm *= nrm;
      if ((rt != 0.0) || (nrm != 0.0)) {
        ztest = std::sqrt(rt * rt + nrm);
        if (rt < 0.0) {
          ztest = -ztest;
        }
        ztest = nrm / (rt + ztest);
      } else {
        ztest = 0.0;
      }
      ztest += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[q] / scale);
      for (k = q + 1; k <= i; k++) {
        blas::xrotg(&ztest, &nrm, &rt, &sqds);
        if (k > q + 1) {
          e[k - 2] = ztest;
        }
        sm = e[k - 1];
        nrm = s_data[k - 1];
        e[k - 1] = rt * sm - sqds * nrm;
        ztest = sqds * s_data[k];
        s_data[k] *= rt;
        s_data[k - 1] = rt * nrm + sqds * sm;
        blas::xrotg(&s_data[k - 1], &ztest, &rt, &sqds);
        ztest = rt * e[k - 1] + sqds * s_data[k];
        s_data[k] = -sqds * e[k - 1] + rt * s_data[k];
        nrm = sqds * e[k];
        e[k] = e[k] * rt;
      }
      e[p] = ztest;
      iter++;
    } break;
    default:
      if (s_data[q] < 0.0) {
        s_data[q] = -s_data[q];
      }
      qp1 = q + 1;
      while ((q + 1 < 7) && (s_data[q] < s_data[qp1])) {
        rt = s_data[q];
        s_data[q] = s_data[qp1];
        s_data[qp1] = rt;
        q = qp1;
        qp1++;
      }
      iter = 0;
      p--;
      break;
    }
  }
  *U_size = 6;
  for (k = 0; k < 6; k++) {
    U_data[k] = s_data[k];
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for svd.cpp
//
// [EOF]
//
