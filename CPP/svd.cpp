//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xrotg.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A[84]
//                double U[6]
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    void svd(const double A[84], double U[6])
    {
      double b_A[84];
      double e[14];
      double s[7];
      double work[6];
      double nrm;
      double rt;
      double sm;
      double snorm;
      double sqds;
      double ztest;
      int b_i;
      int i;
      int ix;
      int iy;
      int k;
      int m;
      int q;
      int qjj;
      int qp1;
      std::memcpy(&b_A[0], &A[0], 84U * sizeof(double));
      for (i = 0; i < 7; i++) {
        s[i] = 0.0;
      }

      std::memset(&e[0], 0, 14U * sizeof(double));
      for (i = 0; i < 6; i++) {
        work[i] = 0.0;
      }

      for (q = 0; q < 6; q++) {
        int nmqp1;
        bool apply_transform;
        qp1 = q + 2;
        m = q + 6 * q;
        i = m + 1;
        nmqp1 = 5 - q;
        apply_transform = false;
        if (q + 1 <= 5) {
          nrm = blas::xnrm2(6 - q, b_A, m + 1);
          if (nrm > 0.0) {
            apply_transform = true;
            if (b_A[m] < 0.0) {
              ztest = -nrm;
              s[q] = -nrm;
            } else {
              ztest = nrm;
              s[q] = nrm;
            }

            if (std::abs(ztest) >= 1.0020841800044864E-292) {
              nrm = 1.0 / ztest;
              b_i = (m - q) + 6;
              for (k = i; k <= b_i; k++) {
                b_A[k - 1] *= nrm;
              }
            } else {
              b_i = (m - q) + 6;
              for (k = i; k <= b_i; k++) {
                b_A[k - 1] /= s[q];
              }
            }

            b_A[m]++;
            s[q] = -s[q];
          } else {
            s[q] = 0.0;
          }
        }

        for (i = qp1; i < 15; i++) {
          qjj = q + 6 * (i - 1);
          if (apply_transform) {
            ix = m + 1;
            iy = qjj;
            nrm = 0.0;
            for (k = 0; k <= nmqp1; k++) {
              nrm += b_A[ix - 1] * b_A[iy];
              ix++;
              iy++;
            }

            nrm = -(nrm / b_A[m]);
            if (!(nrm == 0.0)) {
              ix = m;
              iy = qjj;
              b_i = 5 - q;
              for (k = 0; k <= b_i; k++) {
                b_A[iy] += nrm * b_A[ix];
                ix++;
                iy++;
              }
            }
          }

          e[i - 1] = b_A[qjj];
        }

        nrm = blas::b_xnrm2(13 - q, e, q + 2);
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
            for (k = qp1; k < 15; k++) {
              e[k - 1] *= nrm;
            }
          } else {
            for (k = qp1; k < 15; k++) {
              e[k - 1] /= nrm;
            }
          }

          e[q + 1]++;
          e[q] = -e[q];
          if (q + 2 <= 6) {
            for (qjj = qp1; qjj < 7; qjj++) {
              work[qjj - 1] = 0.0;
            }

            for (i = qp1; i < 15; i++) {
              if (5 - q >= 1) {
                ztest = e[i - 1];
                if (!(ztest == 0.0)) {
                  ix = q + 6 * (i - 1);
                  iy = q + 1;
                  b_i = 4 - q;
                  for (k = 0; k <= b_i; k++) {
                    work[iy] += ztest * b_A[ix + 1];
                    ix++;
                    iy++;
                  }
                }
              }
            }

            for (i = qp1; i < 15; i++) {
              nrm = -e[i - 1] / e[q + 1];
              if ((5 - q >= 1) && (!(nrm == 0.0))) {
                ix = q;
                iy = (q + 6 * (i - 1)) + 1;
                b_i = 4 - q;
                for (k = 0; k <= b_i; k++) {
                  b_A[iy] += nrm * work[ix + 1];
                  ix++;
                  iy++;
                }
              }
            }
          }
        }
      }

      m = 5;
      s[5] = b_A[35];
      s[6] = 0.0;
      e[6] = 0.0;
      iy = 0;
      snorm = 0.0;
      for (q = 0; q < 7; q++) {
        ztest = s[q];
        sm = ztest;
        if (ztest != 0.0) {
          rt = std::abs(ztest);
          sm = rt;
          s[q] = rt;
          if (q + 1 < 7) {
            e[q] /= ztest / rt;
          }
        }

        if ((q + 1 < 7) && (e[q] != 0.0)) {
          rt = std::abs(e[q]);
          nrm = rt / e[q];
          e[q] = rt;
          s[q + 1] *= nrm;
        }

        snorm = std::fmax(snorm, std::fmax(std::abs(sm), std::abs(e[q])));
      }

      while ((m + 2 > 0) && (iy < 75)) {
        qjj = m;
        int exitg1;
        do {
          exitg1 = 0;
          q = qjj + 1;
          if (qjj + 1 == 0) {
            exitg1 = 1;
          } else {
            nrm = std::abs(e[qjj]);
            if ((nrm <= 2.2204460492503131E-16 * (std::abs(s[qjj]) + std::abs
                  (s[qjj + 1]))) || (nrm <= 1.0020841800044864E-292) || ((iy >
                  20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
              e[qjj] = 0.0;
              exitg1 = 1;
            } else {
              qjj--;
            }
          }
        } while (exitg1 == 0);

        if (qjj + 1 == m + 1) {
          i = 4;
        } else {
          bool exitg2;
          ix = m + 2;
          i = m + 2;
          exitg2 = false;
          while ((!exitg2) && (i >= qjj + 1)) {
            ix = i;
            if (i == qjj + 1) {
              exitg2 = true;
            } else {
              nrm = 0.0;
              if (i < m + 2) {
                nrm = std::abs(e[i - 1]);
              }

              if (i > qjj + 2) {
                nrm += std::abs(e[i - 2]);
              }

              ztest = std::abs(s[i - 1]);
              if ((ztest <= 2.2204460492503131E-16 * nrm) || (ztest <=
                   1.0020841800044864E-292)) {
                s[i - 1] = 0.0;
                exitg2 = true;
              } else {
                i--;
              }
            }
          }

          if (ix == qjj + 1) {
            i = 3;
          } else if (ix == m + 2) {
            i = 1;
          } else {
            i = 2;
            q = ix;
          }
        }

        switch (i) {
         case 1:
          ztest = e[m];
          e[m] = 0.0;
          b_i = m + 1;
          for (k = b_i; k >= q + 1; k--) {
            blas::xrotg(&s[k - 1], &ztest, &rt, &sqds);
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
          for (k = q + 1; k <= m + 2; k++) {
            blas::xrotg(&s[k - 1], &ztest, &rt, &sqds);
            sm = e[k - 1];
            ztest = -sqds * sm;
            e[k - 1] = sm * rt;
          }
          break;

         case 3:
          {
            double scale;
            i = m + 1;
            nrm = s[m + 1];
            scale = std::fmax(std::fmax(std::fmax(std::fmax(std::abs(nrm), std::
              abs(s[m])), std::abs(e[m])), std::abs(s[q])), std::abs(e[q]));
            sm = nrm / scale;
            nrm = s[m] / scale;
            ztest = e[m] / scale;
            sqds = s[q] / scale;
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

              nrm = e[k - 1];
              sm = s[k - 1];
              e[k - 1] = rt * nrm - sqds * sm;
              ztest = sqds * s[k];
              s[k] *= rt;
              s[k - 1] = rt * sm + sqds * nrm;
              blas::xrotg(&s[k - 1], &ztest, &rt, &sqds);
              ztest = rt * e[k - 1] + sqds * s[k];
              s[k] = -sqds * e[k - 1] + rt * s[k];
              nrm = sqds * e[k];
              e[k] *= rt;
            }

            e[m] = ztest;
            iy++;
          }
          break;

         default:
          if (s[q] < 0.0) {
            s[q] = -s[q];
          }

          qp1 = q + 1;
          while ((q + 1 < 7) && (s[q] < s[qp1])) {
            rt = s[q];
            s[q] = s[qp1];
            s[qp1] = rt;
            q = qp1;
            qp1++;
          }

          iy = 0;
          m--;
          break;
        }
      }

      for (k = 0; k < 6; k++) {
        U[k] = s[k];
      }
    }
  }
}

//
// File trailer for svd.cpp
//
// [EOF]
//
