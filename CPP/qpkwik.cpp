//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpkwik.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "qpkwik.h"
#include "minOrMax.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>

// Function Declarations
namespace coder
{
  static void DropConstraint(short kDrop, short iA[30], short *nA, short iC[30]);
  static double KWIKfactor(const double Ac[420], const short iC[30], short nA,
    const double Linv[196], double RLinv[196], double D[196], double H[196]);
  static void ResetToColdStart(short iA[30], short iC[30]);
}

static double rt_hypotd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : short kDrop
//                short iA[30]
//                short *nA
//                short iC[30]
// Return Type  : void
//
namespace coder
{
  static void DropConstraint(short kDrop, short iA[30], short *nA, short iC[30])
  {
    int i;
    iA[iC[kDrop - 1] - 1] = 0;
    if (kDrop < *nA) {
      short i1;
      i = *nA - 1;
      if (i < -32768) {
        i = -32768;
      }

      i1 = static_cast<short>(i);
      for (short b_i = kDrop; b_i <= i1; b_i++) {
        iC[b_i - 1] = iC[b_i];
      }
    }

    i = *nA - 1;
    iC[i] = 0;
    if (i < -32768) {
      i = -32768;
    }

    *nA = static_cast<short>(i);
  }

  //
  // Arguments    : const double Ac[420]
  //                const short iC[30]
  //                short nA
  //                const double Linv[196]
  //                double RLinv[196]
  //                double D[196]
  //                double H[196]
  // Return Type  : double
  //
  static double KWIKfactor(const double Ac[420], const short iC[30], short nA,
    const double Linv[196], double RLinv[196], double D[196], double H[196])
  {
    double A[196];
    double Q[196];
    double R[196];
    double TL[196];
    double tau[14];
    double work[14];
    double Status;
    double atmp;
    double beta1;
    double c;
    int b_i;
    int exitg1;
    int i;
    int i1;
    int ia;
    int iac;
    int ii;
    int itau;
    int ix;
    int ix0;
    int knt;
    int lastc;
    int lastv;
    bool exitg2;
    Status = 1.0;
    std::memset(&RLinv[0], 0, 196U * sizeof(double));
    i = nA;
    for (b_i = 0; b_i < i; b_i++) {
      for (i1 = 0; i1 < 14; i1++) {
        knt = i1 + 14 * b_i;
        RLinv[knt] = 0.0;
        for (ix0 = 0; ix0 < 14; ix0++) {
          RLinv[knt] += Linv[i1 + 14 * ix0] * Ac[(iC[b_i] + 30 * ix0) - 1];
        }
      }
    }

    std::memcpy(&A[0], &RLinv[0], 196U * sizeof(double));
    std::memset(&tau[0], 0, 14U * sizeof(double));
    std::memset(&work[0], 0, 14U * sizeof(double));
    for (b_i = 0; b_i < 14; b_i++) {
      ii = b_i * 14 + b_i;
      if (b_i + 1 < 14) {
        atmp = A[ii];
        ix0 = ii + 2;
        tau[b_i] = 0.0;
        c = internal::blas::c_xnrm2(13 - b_i, A, ii + 2);
        if (c != 0.0) {
          double d;
          d = A[ii];
          beta1 = rt_hypotd_snf(d, c);
          if (d >= 0.0) {
            beta1 = -beta1;
          }

          if (std::abs(beta1) < 1.0020841800044864E-292) {
            knt = -1;
            i = (ii - b_i) + 14;
            do {
              knt++;
              for (ix = ix0; ix <= i; ix++) {
                A[ix - 1] *= 9.9792015476736E+291;
              }

              beta1 *= 9.9792015476736E+291;
              atmp *= 9.9792015476736E+291;
            } while (!(std::abs(beta1) >= 1.0020841800044864E-292));

            beta1 = rt_hypotd_snf(atmp, internal::blas::c_xnrm2(13 - b_i, A, ii
              + 2));
            if (atmp >= 0.0) {
              beta1 = -beta1;
            }

            tau[b_i] = (beta1 - atmp) / beta1;
            c = 1.0 / (atmp - beta1);
            for (ix = ix0; ix <= i; ix++) {
              A[ix - 1] *= c;
            }

            for (ix = 0; ix <= knt; ix++) {
              beta1 *= 1.0020841800044864E-292;
            }

            atmp = beta1;
          } else {
            tau[b_i] = (beta1 - d) / beta1;
            c = 1.0 / (d - beta1);
            i = (ii - b_i) + 14;
            for (ix = ix0; ix <= i; ix++) {
              A[ix - 1] *= c;
            }

            atmp = beta1;
          }
        }

        A[ii] = 1.0;
        if (tau[b_i] != 0.0) {
          lastv = 14 - b_i;
          knt = (ii - b_i) + 13;
          while ((lastv > 0) && (A[knt] == 0.0)) {
            lastv--;
            knt--;
          }

          lastc = 13 - b_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            knt = (ii + (lastc - 1) * 14) + 14;
            ia = knt;
            do {
              exitg1 = 0;
              if (ia + 1 <= knt + lastv) {
                if (A[ia] != 0.0) {
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
          knt = ii + 15;
          if (lastc != 0) {
            if (0 <= lastc - 1) {
              std::memset(&work[0], 0, lastc * sizeof(double));
            }

            ix0 = 0;
            i = (ii + 14 * (lastc - 1)) + 15;
            for (iac = knt; iac <= i; iac += 14) {
              ix = ii;
              c = 0.0;
              i1 = (iac + lastv) - 1;
              for (ia = iac; ia <= i1; ia++) {
                c += A[ia - 1] * A[ix];
                ix++;
              }

              work[ix0] += c;
              ix0++;
            }
          }

          internal::blas::xgerc(lastv, lastc, -tau[b_i], ii + 1, work, A, ii +
                                15);
        }

        A[ii] = atmp;
      } else {
        tau[13] = 0.0;
      }
    }

    itau = 13;
    for (ix0 = 0; ix0 < 14; ix0++) {
      for (b_i = 0; b_i <= ix0; b_i++) {
        knt = b_i + 14 * ix0;
        R[knt] = A[knt];
      }

      i = ix0 + 2;
      if (i <= 14) {
        std::memset(&R[(ix0 * 14 + i) + -1], 0, (15 - i) * sizeof(double));
      }

      work[ix0] = 0.0;
    }

    for (b_i = 13; b_i >= 0; b_i--) {
      ii = (b_i + b_i * 14) + 15;
      if (b_i + 1 < 14) {
        A[ii - 15] = 1.0;
        if (tau[itau] != 0.0) {
          lastv = 14 - b_i;
          knt = ii - b_i;
          while ((lastv > 0) && (A[knt - 2] == 0.0)) {
            lastv--;
            knt--;
          }

          lastc = 13 - b_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            knt = ii + (lastc - 1) * 14;
            ia = knt;
            do {
              exitg1 = 0;
              if (ia <= (knt + lastv) - 1) {
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
            if (0 <= lastc - 1) {
              std::memset(&work[0], 0, lastc * sizeof(double));
            }

            ix0 = 0;
            i = ii + 14 * (lastc - 1);
            for (iac = ii; iac <= i; iac += 14) {
              ix = ii;
              c = 0.0;
              i1 = (iac + lastv) - 1;
              for (ia = iac; ia <= i1; ia++) {
                c += A[ia - 1] * A[ix - 15];
                ix++;
              }

              work[ix0] += c;
              ix0++;
            }
          }

          internal::blas::xgerc(lastv, lastc, -tau[itau], ii - 14, work, A, ii);
        }

        ix0 = ii - 13;
        i = (ii - b_i) - 1;
        for (ix = ix0; ix <= i; ix++) {
          A[ix - 1] *= -tau[itau];
        }
      }

      A[ii - 15] = 1.0 - tau[itau];
      for (ix0 = 0; ix0 < b_i; ix0++) {
        A[(ii - ix0) - 16] = 0.0;
      }

      itau--;
    }

    for (ix0 = 0; ix0 < 14; ix0++) {
      std::memcpy(&Q[ix0 * 14], &A[ix0 * 14], 14U * sizeof(double));
    }

    b_i = 0;
    do {
      exitg1 = 0;
      if (b_i <= nA - 1) {
        if (std::abs(R[b_i + 14 * b_i]) < 1.0E-12) {
          Status = -2.0;
          exitg1 = 1;
        } else {
          b_i++;
        }
      } else {
        short j;
        short k;
        for (b_i = 0; b_i < 14; b_i++) {
          for (ix0 = 0; ix0 < 14; ix0++) {
            c = 0.0;
            for (i = 0; i < 14; i++) {
              c += Linv[i + 14 * b_i] * Q[i + 14 * ix0];
            }

            TL[b_i + 14 * ix0] = c;
          }
        }

        std::memset(&RLinv[0], 0, 196U * sizeof(double));
        for (j = nA; j >= 1; j--) {
          i = 14 * (j - 1);
          i1 = (j + i) - 1;
          RLinv[i1] = 1.0;
          for (k = j; k <= nA; k++) {
            knt = (j + 14 * (k - 1)) - 1;
            RLinv[knt] /= R[i1];
          }

          if (j > 1) {
            i1 = j;
            for (b_i = 0; b_i <= i1 - 2; b_i++) {
              for (k = j; k <= nA; k++) {
                knt = 14 * (k - 1);
                ix0 = b_i + knt;
                RLinv[ix0] -= R[b_i + i] * RLinv[(j + knt) - 1];
              }
            }
          }
        }

        for (b_i = 0; b_i < 14; b_i++) {
          short c_i;
          c_i = static_cast<short>(b_i + 1);
          for (j = c_i; j < 15; j++) {
            short i2;
            i = b_i + 14 * (j - 1);
            H[i] = 0.0;
            i1 = nA + 1;
            if (nA + 1 > 32767) {
              i1 = 32767;
            }

            i2 = static_cast<short>(i1);
            for (k = i2; k < 15; k++) {
              i1 = 14 * (k - 1);
              H[i] -= TL[b_i + i1] * TL[(j + i1) - 1];
            }

            H[(j + 14 * b_i) - 1] = H[i];
          }
        }

        i = nA;
        for (ix0 = 0; ix0 < i; ix0++) {
          j = static_cast<short>(ix0 + 1);
          for (b_i = 0; b_i < 14; b_i++) {
            i1 = b_i + 14 * ix0;
            D[i1] = 0.0;
            for (k = j; k <= nA; k++) {
              knt = 14 * (k - 1);
              D[i1] += TL[b_i + knt] * RLinv[ix0 + knt];
            }
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    return Status;
  }

  //
  // Arguments    : short iA[30]
  //                short iC[30]
  // Return Type  : void
  //
  static void ResetToColdStart(short iA[30], short iC[30])
  {
    int i;
    for (i = 0; i < 30; i++) {
      iA[i] = 0;
      iC[i] = 0;
    }

    for (i = 0; i < 6; i++) {
      iA[i + 24] = 1;
      iC[i] = static_cast<short>(i + 25);
    }
  }

  //
  // Arguments    : double u0
  //                double u1
  // Return Type  : double
  //
}

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
  } else {
    if (!std::isnan(y)) {
      y = a * 1.4142135623730951;
    }
  }

  return y;
}

//
// Arguments    : const double Linv[196]
//                const double Hinv[196]
//                const double f[14]
//                const double Ac[420]
//                const double b[30]
//                short iA[30]
//                double x[14]
//                double lambda[30]
//                double *status
// Return Type  : void
//
namespace coder
{
  void qpkwik(const double Linv[196], const double Hinv[196], const double f[14],
              const double Ac[420], const double b[30], short iA[30], double x
              [14], double lambda[30], double *status)
  {
    double D[196];
    double H[196];
    double RLinv[196];
    double U[196];
    double cTol[30];
    double Opt[28];
    double Rhs[28];
    double r[14];
    double z[14];
    double b_Ac;
    double cVal;
    double rMin;
    int b_i;
    int ct;
    int i;
    int i1;
    int k;
    short iC[30];
    short c_i;
    short kDrop;
    short nA;
    bool ColdReset;
    bool DualFeasible;
    bool cTolComputed;
    bool guard1{
      false
    };

    *status = 1.0;
    std::memset(&lambda[0], 0, 30U * sizeof(double));
    std::memset(&x[0], 0, 14U * sizeof(double));
    std::memset(&r[0], 0, 14U * sizeof(double));
    rMin = 0.0;
    cTolComputed = false;
    for (i = 0; i < 30; i++) {
      cTol[i] = 1.0;
      iC[i] = 0;
    }

    nA = 0;
    for (i = 0; i < 30; i++) {
      if (iA[i] == 1) {
        nA = static_cast<short>(nA + 1);
        iC[nA - 1] = static_cast<short>(i + 1);
      }
    }

    guard1 = false;
    if (nA > 0) {
      int exitg3;
      unsigned short b_x;
      unsigned short q;
      std::memset(&Opt[0], 0, 28U * sizeof(double));
      for (i = 0; i < 14; i++) {
        Rhs[i] = f[i];
        Rhs[i + 14] = 0.0;
      }

      DualFeasible = false;
      kDrop = static_cast<short>(3 * nA);
      if (kDrop <= 50) {
        kDrop = 50;
      }

      q = static_cast<unsigned short>(static_cast<unsigned short>(kDrop) / 10U);
      b_x = static_cast<unsigned short>(static_cast<unsigned int>(static_cast<
        unsigned short>(kDrop)) - static_cast<unsigned short>(q * 10));
      if ((b_x > 0) && (b_x >= 5)) {
        q = static_cast<unsigned short>(q + 1);
      }

      ColdReset = false;
      do {
        exitg3 = 0;
        if ((!DualFeasible) && (nA > 0) && (*status <= 200.0)) {
          cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
          if (cVal < 0.0) {
            if (ColdReset) {
              *status = -2.0;
              exitg3 = 2;
            } else {
              ResetToColdStart(iA, iC);
              nA = 6;
              ColdReset = true;
            }
          } else {
            b_i = nA;
            for (int j = 0; j < b_i; j++) {
              kDrop = static_cast<short>(j + 1);
              i1 = j + 15;
              if (j + 15 > 32767) {
                i1 = 32767;
              }

              Rhs[static_cast<short>(i1) - 1] = b[iC[j] - 1];
              for (c_i = kDrop; c_i <= nA; c_i++) {
                ct = (c_i + 14 * j) - 1;
                U[ct] = 0.0;
                i1 = nA;
                for (k = 0; k < i1; k++) {
                  i = 14 * k;
                  U[ct] += RLinv[(c_i + i) - 1] * RLinv[j + i];
                }

                U[j + 14 * (c_i - 1)] = U[ct];
              }
            }

            b_i = nA;
            for (i = 0; i < 14; i++) {
              cVal = 0.0;
              for (i1 = 0; i1 < 14; i1++) {
                cVal += H[i + 14 * i1] * Rhs[i1];
              }

              Opt[i] = cVal;
              for (k = 0; k < b_i; k++) {
                i1 = k + 15;
                if (k + 15 > 32767) {
                  i1 = 32767;
                }

                Opt[i] += D[i + 14 * k] * Rhs[static_cast<short>(i1) - 1];
              }
            }

            b_i = nA;
            for (i = 0; i < b_i; i++) {
              cVal = 0.0;
              for (i1 = 0; i1 < 14; i1++) {
                cVal += D[i1 + 14 * i] * Rhs[i1];
              }

              Opt[i + 14] = cVal;
              i1 = nA;
              for (k = 0; k < i1; k++) {
                ct = k + 15;
                if (k + 15 > 32767) {
                  ct = 32767;
                }

                Opt[i + 14] += U[i + 14 * k] * Rhs[static_cast<short>(ct) - 1];
              }
            }

            cVal = -1.0E-12;
            kDrop = 0;
            b_i = nA;
            for (i = 0; i < b_i; i++) {
              i1 = i + 15;
              if (i + 15 > 32767) {
                i1 = 32767;
              }

              lambda[iC[i] - 1] = Opt[static_cast<short>(i1) - 1];
              i1 = i + 15;
              if (i + 15 > 32767) {
                i1 = 32767;
              }

              if ((Opt[static_cast<short>(i1) - 1] < cVal) && (i + 1 <= nA - 6))
              {
                kDrop = static_cast<short>(i + 1);
                i1 = i + 15;
                if (i + 15 > 32767) {
                  i1 = 32767;
                }

                cVal = Opt[static_cast<short>(i1) - 1];
              }
            }

            if (kDrop <= 0) {
              DualFeasible = true;
              std::memcpy(&x[0], &Opt[0], 14U * sizeof(double));
            } else {
              (*status)++;
              if (static_cast<int>(*status) > q) {
                ResetToColdStart(iA, iC);
                nA = 6;
                ColdReset = true;
              } else {
                lambda[iC[kDrop - 1] - 1] = 0.0;
                DropConstraint(kDrop, iA, &nA, iC);
              }
            }
          }
        } else {
          if (nA <= 0) {
            std::memset(&lambda[0], 0, 30U * sizeof(double));
            for (i = 0; i < 14; i++) {
              cVal = 0.0;
              for (b_i = 0; b_i < 14; b_i++) {
                cVal += -Hinv[i + 14 * b_i] * f[b_i];
              }

              x[i] = cVal;
            }
          }

          exitg3 = 1;
        }
      } while (exitg3 == 0);

      if (exitg3 == 1) {
        guard1 = true;
      }
    } else {
      for (i = 0; i < 14; i++) {
        cVal = 0.0;
        for (b_i = 0; b_i < 14; b_i++) {
          cVal += -Hinv[i + 14 * b_i] * f[b_i];
        }

        x[i] = cVal;
      }

      guard1 = true;
    }

    if (guard1) {
      double Xnorm0;
      bool exitg2;
      Xnorm0 = b_norm(x);
      exitg2 = false;
      while ((!exitg2) && (*status <= 200.0)) {
        double cMin;
        short kNext;
        cMin = -1.0E-6;
        kNext = 0;
        for (i = 0; i < 24; i++) {
          if (!cTolComputed) {
            for (k = 0; k < 14; k++) {
              z[k] = std::abs(Ac[i + 30 * k] * x[k]);
            }

            cTol[i] = std::fmax(cTol[i], internal::maximum(z));
          }

          if (iA[i] == 0) {
            b_Ac = 0.0;
            for (b_i = 0; b_i < 14; b_i++) {
              b_Ac += Ac[i + 30 * b_i] * x[b_i];
            }

            cVal = (b_Ac - b[i]) / cTol[i];
            if (cVal < cMin) {
              cMin = cVal;
              kNext = static_cast<short>(i + 1);
            }
          }
        }

        cTolComputed = true;
        if (kNext <= 0) {
          exitg2 = true;
        } else if (*status == 200.0) {
          *status = 0.0;
          exitg2 = true;
        } else {
          int exitg1;
          do {
            exitg1 = 0;
            if ((kNext > 0) && (*status <= 200.0)) {
              bool guard2{
                false
              };

              guard2 = false;
              if (nA == 0) {
                for (b_i = 0; b_i < 14; b_i++) {
                  cVal = 0.0;
                  for (i1 = 0; i1 < 14; i1++) {
                    cVal += Hinv[b_i + 14 * i1] * Ac[(kNext + 30 * i1) - 1];
                  }

                  z[b_i] = cVal;
                }

                guard2 = true;
              } else {
                cVal = KWIKfactor(Ac, iC, nA, Linv, RLinv, D, H);
                if (cVal <= 0.0) {
                  *status = -2.0;
                  exitg1 = 1;
                } else {
                  for (b_i = 0; b_i < 196; b_i++) {
                    U[b_i] = -H[b_i];
                  }

                  for (b_i = 0; b_i < 14; b_i++) {
                    cVal = 0.0;
                    for (i1 = 0; i1 < 14; i1++) {
                      cVal += U[b_i + 14 * i1] * Ac[(kNext + 30 * i1) - 1];
                    }

                    z[b_i] = cVal;
                  }

                  b_i = nA;
                  for (i = 0; i < b_i; i++) {
                    b_Ac = 0.0;
                    for (i1 = 0; i1 < 14; i1++) {
                      b_Ac += Ac[(kNext + 30 * i1) - 1] * D[i1 + 14 * i];
                    }

                    r[i] = b_Ac;
                  }

                  guard2 = true;
                }
              }

              if (guard2) {
                double t1;
                bool exitg4;
                kDrop = 0;
                t1 = 0.0;
                ColdReset = true;
                DualFeasible = true;
                if (nA > 6) {
                  ct = 0;
                  exitg4 = false;
                  while ((!exitg4) && (ct <= nA - 7)) {
                    if (r[ct] >= 1.0E-12) {
                      DualFeasible = false;
                      exitg4 = true;
                    } else {
                      ct++;
                    }
                  }
                }

                if ((nA != 6) && (!DualFeasible)) {
                  b_i = nA - 6;
                  if (nA - 6 < -32768) {
                    b_i = -32768;
                  }

                  b_i = static_cast<short>(b_i);
                  for (i = 0; i < b_i; i++) {
                    cVal = r[i];
                    if (cVal > 1.0E-12) {
                      cVal = lambda[iC[i] - 1] / cVal;
                      if ((kDrop == 0) || (cVal < rMin)) {
                        rMin = cVal;
                        kDrop = static_cast<short>(i + 1);
                      }
                    }
                  }

                  if (kDrop > 0) {
                    t1 = rMin;
                    ColdReset = false;
                  }
                }

                cVal = 0.0;
                for (k = 0; k < 14; k++) {
                  cVal += z[k] * Ac[(kNext + 30 * k) - 1];
                }

                if (cVal <= 0.0) {
                  cVal = 0.0;
                  DualFeasible = true;
                } else {
                  b_Ac = 0.0;
                  for (b_i = 0; b_i < 14; b_i++) {
                    b_Ac += Ac[(kNext + 30 * b_i) - 1] * x[b_i];
                  }

                  cVal = (b[kNext - 1] - b_Ac) / cVal;
                  DualFeasible = false;
                }

                if (ColdReset && DualFeasible) {
                  *status = -1.0;
                  exitg1 = 1;
                } else {
                  if (DualFeasible) {
                    cMin = t1;
                  } else if (ColdReset) {
                    cMin = cVal;
                  } else {
                    cMin = std::fmin(t1, cVal);
                  }

                  b_i = nA;
                  for (i = 0; i < b_i; i++) {
                    ct = iC[i];
                    lambda[ct - 1] -= cMin * r[i];
                    if ((ct <= 24) && (lambda[ct - 1] < 0.0)) {
                      lambda[ct - 1] = 0.0;
                    }
                  }

                  lambda[kNext - 1] += cMin;
                  if (cMin == t1) {
                    DropConstraint(kDrop, iA, &nA, iC);
                  }

                  if (!DualFeasible) {
                    for (b_i = 0; b_i < 14; b_i++) {
                      x[b_i] += cMin * z[b_i];
                    }

                    if (cMin == cVal) {
                      if (nA == 14) {
                        *status = -1.0;
                        exitg1 = 1;
                      } else {
                        b_i = nA + 1;
                        if (nA + 1 > 32767) {
                          b_i = 32767;
                        }

                        nA = static_cast<short>(b_i);
                        iC[static_cast<short>(b_i) - 1] = kNext;
                        c_i = static_cast<short>(b_i);
                        exitg4 = false;
                        while ((!exitg4) && (c_i > 1)) {
                          short i2;
                          kDrop = iC[c_i - 1];
                          i2 = iC[c_i - 2];
                          if (kDrop > i2) {
                            exitg4 = true;
                          } else {
                            iC[c_i - 1] = i2;
                            iC[c_i - 2] = kDrop;
                            c_i = static_cast<short>(c_i - 1);
                          }
                        }

                        iA[kNext - 1] = 1;
                        kNext = 0;
                        (*status)++;
                      }
                    } else {
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                }
              }
            } else {
              cVal = b_norm(x);
              if (std::abs(cVal - Xnorm0) > 0.001) {
                Xnorm0 = cVal;
                for (k = 0; k < 30; k++) {
                  cTol[k] = std::fmax(std::abs(b[k]), 1.0);
                }

                cTolComputed = false;
              }

              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      }
    }
  }
}

//
// File trailer for qpkwik.cpp
//
// [EOF]
//
