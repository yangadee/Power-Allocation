// 
// Academic License - for use in teaching, academic research, and meeting 
// course requirements at degree granting institutions only.  Not for 
// government, commercial, or other organizational use. 
// File: qpkwik.cpp 
//  
// MATLAB Coder version            : 5.2 
// C/C++ source code generated on  : 14-Mar-2021 05:17:40 
//

// Include Files 
#include "qpkwik.h"
#include "minOrMax.h"
#include "mtimes.h"
#include "norm.h"
#include "rt_nonfinite.h"
#include "xgeqrf.h"
#include "xorgqr.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Declarations 
namespace coder {
static void DropConstraint(short kDrop, ::coder::array<short, 1U> &iA, short *nA, short iC_data[]);

static double KWIKfactor(const ::coder::array<double, 2U> &Ac, const short iC_data[], short nA, const ::coder::array<double, 2U> &Linv, ::coder::array<double, 2U> &RLinv, ::coder::array<double, 2U> &D, ::coder::array<double, 2U> &H, short n);

static void ResetToColdStart(short m, short iA_data[], int *iA_size, short iC_data[], int *iC_size);

static void Unconstrained(const ::coder::array<double, 2U> &Hinv, const ::coder::array<double, 1U> &f, ::coder::array<double, 1U> &x, short n);

}

// Function Definitions 
// 
// Arguments    : short kDrop 
//                ::coder::array<short, 1U> &iA 
//                short *nA 
//                short iC_data[] 
// Return Type  : void 
//
namespace coder {
static void DropConstraint(short kDrop, ::coder::array<short, 1U> &iA, short *nA, short iC_data[])
{
    int i;
    iA[iC_data[kDrop - 1] - 1] = 0;
    if (kDrop < *nA) {
        short i1;
        i = *nA - 1;
        if (i < -32768) {
            i = -32768;
        }
        i1 = static_cast<short>(i);
        for ( short b_i{kDrop}; b_i <= i1; b_i++) {
            iC_data[b_i - 1] = iC_data[b_i];
        }
    }
    i = *nA - 1;
    iC_data[i] = 0;
    if (i < -32768) {
        i = -32768;
    }
    *nA = static_cast<short>(i);
}

// 
// Arguments    : const ::coder::array<double, 2U> &Ac 
//                const short iC_data[] 
//                short nA 
//                const ::coder::array<double, 2U> &Linv 
//                ::coder::array<double, 2U> &RLinv 
//                ::coder::array<double, 2U> &D 
//                ::coder::array<double, 2U> &H 
//                short n 
// Return Type  : double 
//
static double KWIKfactor(const ::coder::array<double, 2U> &Ac, const short iC_data[], short nA, const ::coder::array<double, 2U> &Linv, ::coder::array<double, 2U> &RLinv, ::coder::array<double, 2U> &D, ::coder::array<double, 2U> &H, short n)
{
    array<double, 2U> A;
    array<double, 2U> B;
    array<double, 2U> Q;
    array<double, 2U> R;
    array<double, 2U> TL;
    array<double, 1U> C;
    double Status;
    int aoffset;
    int b_i;
    int b_n;
    int c_i;
    int i;
    int i1;
    int k;
    int m;
    TL.set_size(Linv.size(0), Linv.size(1));
    Status = 1.0;
    m = RLinv.size(0);
    aoffset = RLinv.size(1);
    for (i = 0; i < aoffset; i++) {
        for (i1 = 0; i1 < m; i1++) {
            RLinv[i1 + RLinv.size(0) * i] = 0.0;
        }
    }
    i = nA;
    for (b_i = 0; b_i < i; b_i++) {
        aoffset = Ac.size(1);
        B.set_size(1, Ac.size(1));
        for (i1 = 0; i1 < aoffset; i1++) {
            B[i1] = Ac[(iC_data[b_i] + Ac.size(0) * i1) - 1];
        }
        m = Linv.size(0) - 1;
        b_n = Linv.size(1);
        C.set_size(Linv.size(0));
        for (c_i = 0; c_i <= m; c_i++) {
            C[c_i] = 0.0;
        }
        for (k = 0; k < b_n; k++) {
            aoffset = k * Linv.size(0);
            for (c_i = 0; c_i <= m; c_i++) {
                C[c_i] = C[c_i] + Linv[aoffset + c_i] * B[k];
            }
        }
        aoffset = C.size(0);
        for (i1 = 0; i1 < aoffset; i1++) {
            RLinv[i1 + RLinv.size(0) * b_i] = C[i1];
        }
    }
    m = RLinv.size(0) - 1;
    b_n = RLinv.size(1);
    Q.set_size(RLinv.size(0), RLinv.size(0));
    R.set_size(RLinv.size(0), RLinv.size(1));
    if (RLinv.size(0) > RLinv.size(1)) {
        for (c_i = 0; c_i < b_n; c_i++) {
            for (b_i = 0; b_i <= m; b_i++) {
                Q[b_i + Q.size(0) * c_i] = RLinv[b_i + RLinv.size(0) * c_i];
            }
        }
        i1 = RLinv.size(1) + 1;
        for (c_i = i1; c_i <= m + 1; c_i++) {
            for (b_i = 0; b_i <= m; b_i++) {
                Q[b_i + Q.size(0) * (c_i - 1)] = 0.0;
            }
        }
        internal::lapack::xgeqrf(Q, C);
        for (c_i = 0; c_i < b_n; c_i++) {
            for (b_i = 0; b_i <= c_i; b_i++) {
                R[b_i + R.size(0) * c_i] = Q[b_i + Q.size(0) * c_i];
            }
            i1 = c_i + 2;
            for (b_i = i1; b_i <= m + 1; b_i++) {
                R[(b_i + R.size(0) * c_i) - 1] = 0.0;
            }
        }
        internal::lapack::xorgqr(RLinv.size(0), RLinv.size(0), RLinv.size(1), Q, RLinv.size(0), C);
    } else {
        A.set_size(RLinv.size(0), RLinv.size(1));
        aoffset = RLinv.size(0) * RLinv.size(1);
        for (i1 = 0; i1 < aoffset; i1++) {
            A[i1] = RLinv[i1];
        }
        internal::lapack::xgeqrf(A, C);
        for (c_i = 0; c_i <= m; c_i++) {
            for (b_i = 0; b_i <= c_i; b_i++) {
                R[b_i + R.size(0) * c_i] = A[b_i + A.size(0) * c_i];
            }
            i1 = c_i + 2;
            for (b_i = i1; b_i <= m + 1; b_i++) {
                R[(b_i + R.size(0) * c_i) - 1] = 0.0;
            }
        }
        i1 = RLinv.size(0) + 1;
        for (c_i = i1; c_i <= b_n; c_i++) {
            for (b_i = 0; b_i <= m; b_i++) {
                R[b_i + R.size(0) * (c_i - 1)] = A[b_i + A.size(0) * (c_i - 1)];
            }
        }
        internal::lapack::xorgqr(RLinv.size(0), RLinv.size(0), RLinv.size(0), A, RLinv.size(0), C);
        for (c_i = 0; c_i <= m; c_i++) {
            for (b_i = 0; b_i <= m; b_i++) {
                Q[b_i + Q.size(0) * c_i] = A[b_i + A.size(0) * c_i];
            }
        }
    }
    b_i = 0;
    int exitg1;
    do {
        exitg1 = 0;
        if (b_i <= nA - 1) {
            if (std::abs(R[b_i + R.size(0) * b_i]) < 1.0E-12) {
                Status = -2.0;
                exitg1 = 1;
            } else {
                b_i++;
            }
        } else {
            short b_k;
            short j;
            i1 = n;
            for (b_i = 0; b_i < i1; b_i++) {
                for (c_i = 0; c_i < i1; c_i++) {
                    double b_Linv;
                    aoffset = Linv.size(0);
                    b_Linv = 0.0;
                    for (k = 0; k < aoffset; k++) {
                        b_Linv += Linv[k + Linv.size(0) * b_i] * Q[k + Q.size(0) * c_i];
                    }
                    TL[b_i + TL.size(0) * c_i] = b_Linv;
                }
            }
            m = RLinv.size(0);
            aoffset = RLinv.size(1);
            for (k = 0; k < aoffset; k++) {
                for (b_n = 0; b_n < m; b_n++) {
                    RLinv[b_n + RLinv.size(0) * k] = 0.0;
                }
            }
            for (j = nA; j >= 1; j--) {
                RLinv[(j + RLinv.size(0) * (j - 1)) - 1] = 1.0;
                for (b_k = j; b_k <= nA; b_k++) {
                    RLinv[(j + RLinv.size(0) * (b_k - 1)) - 1] = RLinv[(j + RLinv.size(0) * (b_k - 1)) - 1] / R[(j + R.size(0) * (j - 1)) - 1];
                }
                if (j > 1) {
                    k = j;
                    for (b_i = 0; b_i <= k - 2; b_i++) {
                        for (b_k = j; b_k <= nA; b_k++) {
                            RLinv[b_i + RLinv.size(0) * (b_k - 1)] = RLinv[b_i + RLinv.size(0) * (b_k - 1)] - R[b_i + R.size(0) * (j - 1)] * RLinv[(j + RLinv.size(0) * (b_k - 1)) - 1];
                        }
                    }
                }
            }
            for (b_i = 0; b_i < i1; b_i++) {
                short d_i;
                d_i = static_cast<short>(b_i + 1);
                for (j = d_i; j <= n; j++) {
                    short i2;
                    H[b_i + H.size(0) * (j - 1)] = 0.0;
                    k = nA + 1;
                    if (nA + 1 > 32767) {
                        k = 32767;
                    }
                    i2 = static_cast<short>(k);
                    for (b_k = i2; b_k <= n; b_k++) {
                        H[b_i + H.size(0) * (j - 1)] = H[b_i + H.size(0) * (j - 1)] - TL[b_i + TL.size(0) * (b_k - 1)] * TL[(j + TL.size(0) * (b_k - 1)) - 1];
                    }
                    H[(j + H.size(0) * b_i) - 1] = H[b_i + H.size(0) * (j - 1)];
                }
            }
            for (c_i = 0; c_i < i; c_i++) {
                j = static_cast<short>(c_i + 1);
                for (b_i = 0; b_i < i1; b_i++) {
                    D[b_i + D.size(0) * c_i] = 0.0;
                    for (b_k = j; b_k <= nA; b_k++) {
                        D[b_i + D.size(0) * c_i] = D[b_i + D.size(0) * c_i] + TL[b_i + TL.size(0) * (b_k - 1)] * RLinv[c_i + RLinv.size(0) * (b_k - 1)];
                    }
                }
            }
            exitg1 = 1;
        }
    } while (exitg1 == 0);
    return Status;
}

// 
// Arguments    : short m 
//                short iA_data[] 
//                int *iA_size 
//                short iC_data[] 
//                int *iC_size 
// Return Type  : void 
//
static void ResetToColdStart(short m, short iA_data[], int *iA_size, short iC_data[], int *iC_size)
{
    int iA_size_tmp;
    iA_size_tmp = m;
    *iA_size = m;
    if (0 <= iA_size_tmp - 1) {
        std::memset(&iA_data[0], 0, iA_size_tmp * sizeof(short));
    }
    *iC_size = m;
    if (0 <= iA_size_tmp - 1) {
        std::memset(&iC_data[0], 0, iA_size_tmp * sizeof(short));
    }
    for (iA_size_tmp = 0; iA_size_tmp < 6; iA_size_tmp++) {
        int i;
        short ix;
        i = m - 6;
        if (m - 6 < -32768) {
            i = -32768;
        }
        ix = static_cast<short>((static_cast<short>(i) + iA_size_tmp) + 1);
        iA_data[ix - 1] = 1;
        iC_data[iA_size_tmp] = ix;
    }
}

// 
// Arguments    : const ::coder::array<double, 2U> &Hinv 
//                const ::coder::array<double, 1U> &f 
//                ::coder::array<double, 1U> &x 
//                short n 
// Return Type  : void 
//
static void Unconstrained(const ::coder::array<double, 2U> &Hinv, const ::coder::array<double, 1U> &f, ::coder::array<double, 1U> &x, short n)
{
    int i;
    i = n;
    for ( int b_i{0}; b_i < i; b_i++) {
        double b_Hinv;
        int loop_ub;
        loop_ub = Hinv.size(1);
        b_Hinv = 0.0;
        for ( int i1{0}; i1 < loop_ub; i1++) {
            b_Hinv += -Hinv[b_i + Hinv.size(0) * i1] * f[i1];
        }
        x[b_i] = b_Hinv;
    }
}

// 
// Arguments    : const ::coder::array<double, 2U> &Linv 
//                const ::coder::array<double, 2U> &Hinv 
//                const ::coder::array<double, 1U> &f 
//                const ::coder::array<double, 2U> &Ac 
//                const ::coder::array<double, 1U> &b 
//                ::coder::array<short, 1U> &iA 
//                short m 
//                short n 
//                ::coder::array<double, 1U> &x 
//                ::coder::array<double, 1U> &lambda 
//                double *status 
// Return Type  : void 
//
void qpkwik(const ::coder::array<double, 2U> &Linv, const ::coder::array<double, 2U> &Hinv, const ::coder::array<double, 1U> &f, const ::coder::array<double, 2U> &Ac, const ::coder::array<double, 1U> &b, ::coder::array<short, 1U> &iA, short m, short n, ::coder::array<double, 1U> &x, ::coder::array<double, 1U> &lambda, double *status)
{
    array<double, 2U> AcRow;
    array<double, 2U> D;
    array<double, 2U> H;
    array<double, 2U> RLinv;
    array<double, 2U> U;
    array<double, 2U> b_Ac;
    array<double, 2U> b_r;
    array<double, 2U> d_Ac;
    array<double, 1U> Opt;
    array<double, 1U> Rhs;
    array<double, 1U> cTol;
    array<double, 1U> r;
    double cVal;
    double rMin;
    int aoffset;
    int b_i;
    int b_status;
    int i;
    int i1;
    int inner;
    int k;
    int nx;
    short iA_data[32767];
    short iC_data[32767];
    short c_i;
    short kDrop;
    short nA;
    boolean_T ColdReset;
    boolean_T DualFeasible;
    boolean_T cTolComputed;
    boolean_T guard1{false};
    b_status = 1;
    lambda.set_size(static_cast<int>(m));
    nx = m;
    for (i = 0; i < nx; i++) {
        lambda[i] = 0.0;
    }
    x.set_size(static_cast<int>(n));
    nx = n;
    for (i = 0; i < nx; i++) {
        x[i] = 0.0;
    }
    r.set_size(static_cast<int>(n));
    nx = n;
    for (i = 0; i < nx; i++) {
        r[i] = 0.0;
    }
    rMin = 0.0;
    RLinv.set_size(Linv.size(0), Linv.size(1));
    D.set_size(Linv.size(0), Linv.size(1));
    H.set_size(Linv.size(0), Linv.size(1));
    U.set_size(Linv.size(0), Linv.size(1));
    cTol.set_size(static_cast<int>(m));
    nx = m;
    for (i = 0; i < nx; i++) {
        cTol[i] = 1.0;
    }
    cTolComputed = false;
    nx = m;
    if (0 <= nx - 1) {
        std::memset(&iC_data[0], 0, nx * sizeof(short));
    }
    nA = 0;
    i = m;
    for (b_i = 0; b_i < i; b_i++) {
        if (iA[b_i] == 1) {
            i1 = nA + 1;
            if (nA + 1 > 32767) {
                i1 = 32767;
            }
            nA = static_cast<short>(i1);
            iC_data[static_cast<short>(i1) - 1] = static_cast<short>(b_i + 1);
        }
    }
    guard1 = false;
    if (nA > 0) {
        int exitg3;
        unsigned short b_x;
        unsigned short q;
        if (n > 16383) {
            kDrop = MAX_int16_T;
        } else if (n <= -16384) {
            kDrop = MIN_int16_T;
        } else {
            kDrop = static_cast<short>(n << 1);
        }
        Opt.set_size(static_cast<int>(kDrop));
        nx = kDrop;
        for (i = 0; i < nx; i++) {
            Opt[i] = 0.0;
        }
        Rhs.set_size(f.size(0) + n);
        nx = f.size(0);
        for (i = 0; i < nx; i++) {
            Rhs[i] = f[i];
        }
        nx = n;
        for (i = 0; i < nx; i++) {
            Rhs[i + f.size(0)] = 0.0;
        }
        DualFeasible = false;
        i = 3 * nA;
        if (i > 32767) {
            i = 32767;
        }
        if (i > 50) {
            kDrop = static_cast<short>(i);
        } else {
            kDrop = 50;
        }
        q = static_cast<unsigned short>(kDrop / 10U);
        b_x = static_cast<unsigned short>(static_cast<unsigned int>(kDrop) - q * 10);
        if ((b_x > 0) && (b_x >= 5)) {
            q = static_cast<unsigned short>(q + 1);
        }
        ColdReset = false;
        do {
            exitg3 = 0;
            if ((!DualFeasible) && (nA > 0) && (b_status <= 200)) {
                cVal = KWIKfactor(Ac, iC_data, nA, Linv, RLinv, D, H, n);
                if (cVal < 0.0) {
                    if (ColdReset) {
                        b_status = -2;
                        exitg3 = 2;
                    } else {
                        ResetToColdStart(m, iA_data, &inner, iC_data, &nx);
                        nA = 6;
                        iA.set_size(inner);
                        for (i = 0; i < inner; i++) {
                            iA[i] = iA_data[i];
                        }
                        ColdReset = true;
                    }
                } else {
                    i = nA;
                    for (nx = 0; nx < i; nx++) {
                        kDrop = static_cast<short>(nx + 1);
                        i1 = (n + nx) + 1;
                        if (i1 > 32767) {
                            i1 = 32767;
                        }
                        Rhs[i1 - 1] = b[iC_data[nx] - 1];
                        for (c_i = kDrop; c_i <= nA; c_i++) {
                            U[(c_i + U.size(0) * nx) - 1] = 0.0;
                            i1 = nA;
                            for (k = 0; k < i1; k++) {
                                U[(c_i + U.size(0) * nx) - 1] = U[(c_i + U.size(0) * nx) - 1] + RLinv[(c_i + RLinv.size(0) * k) - 1] * RLinv[nx + RLinv.size(0) * k];
                            }
                            U[nx + U.size(0) * (c_i - 1)] = U[(c_i + U.size(0) * nx) - 1];
                        }
                    }
                    i = n;
                    for (b_i = 0; b_i < i; b_i++) {
                        nx = H.size(1);
                        cVal = 0.0;
                        for (i1 = 0; i1 < nx; i1++) {
                            cVal += H[b_i + H.size(0) * i1] * Rhs[i1];
                        }
                        Opt[b_i] = cVal;
                        i1 = nA;
                        for (k = 0; k < i1; k++) {
                            nx = (n + k) + 1;
                            if (nx > 32767) {
                                nx = 32767;
                            }
                            Opt[b_i] = Opt[b_i] + D[b_i + D.size(0) * k] * Rhs[nx - 1];
                        }
                    }
                    i = nA;
                    for (b_i = 0; b_i < i; b_i++) {
                        nx = D.size(0);
                        cVal = 0.0;
                        for (i1 = 0; i1 < nx; i1++) {
                            cVal += D[i1 + D.size(0) * b_i] * Rhs[i1];
                        }
                        i1 = (n + b_i) + 1;
                        nx = i1;
                        if (i1 > 32767) {
                            nx = 32767;
                        }
                        Opt[static_cast<short>(nx) - 1] = cVal;
                        nx = nA;
                        for (k = 0; k < nx; k++) {
                            int i2;
                            inner = i1;
                            if (i1 > 32767) {
                                inner = 32767;
                            }
                            aoffset = i1;
                            if (i1 > 32767) {
                                aoffset = 32767;
                            }
                            i2 = (n + k) + 1;
                            if (i2 > 32767) {
                                i2 = 32767;
                            }
                            Opt[static_cast<short>(inner) - 1] = Opt[static_cast<short>(aoffset) - 1] + U[b_i + U.size(0) * k] * Rhs[i2 - 1];
                        }
                    }
                    cVal = -1.0E-12;
                    kDrop = 0;
                    i = nA;
                    for (b_i = 0; b_i < i; b_i++) {
                        i1 = (n + b_i) + 1;
                        nx = i1;
                        if (i1 > 32767) {
                            nx = 32767;
                        }
                        lambda[iC_data[b_i] - 1] = Opt[static_cast<short>(nx) - 1];
                        nx = i1;
                        if (i1 > 32767) {
                            nx = 32767;
                        }
                        if ((Opt[static_cast<short>(nx) - 1] < cVal) && (b_i + 1 <= nA - 6)) {
                            kDrop = static_cast<short>(b_i + 1);
                            if (i1 > 32767) {
                                i1 = 32767;
                            }
                            cVal = Opt[static_cast<short>(i1) - 1];
                        }
                    }
                    if (kDrop <= 0) {
                        DualFeasible = true;
                        nx = n;
                        x.set_size(static_cast<int>(n));
                        for (i = 0; i < nx; i++) {
                            x[i] = Opt[i];
                        }
                    } else {
                        b_status++;
                        if (b_status > q) {
                            ResetToColdStart(m, iA_data, &inner, iC_data, &nx);
                            nA = 6;
                            iA.set_size(inner);
                            for (i = 0; i < inner; i++) {
                                iA[i] = iA_data[i];
                            }
                            ColdReset = true;
                        } else {
                            lambda[iC_data[kDrop - 1] - 1] = 0.0;
                            DropConstraint(kDrop, iA, &nA, iC_data);
                        }
                    }
                }
            } else {
                if (nA <= 0) {
                    lambda.set_size(static_cast<int>(m));
                    nx = m;
                    for (i = 0; i < nx; i++) {
                        lambda[i] = 0.0;
                    }
                    Unconstrained(Hinv, f, x, n);
                }
                exitg3 = 1;
            }
        } while (exitg3 == 0);
        if (exitg3 == 1) {
            guard1 = true;
        }
    } else {
        Unconstrained(Hinv, f, x, n);
        guard1 = true;
    }
    if (guard1) {
        double Xnorm0;
        boolean_T exitg2;
        Xnorm0 = b_norm(x);
        exitg2 = false;
        while ((!exitg2) && (b_status <= 200)) {
            double cMin;
            double c_Ac;
            short kNext;
            cMin = -1.0E-6;
            kNext = 0;
            i = m - 6;
            if (m - 6 < -32768) {
                i = -32768;
            }
            i = static_cast<short>(i);
            for (b_i = 0; b_i < i; b_i++) {
                if (!cTolComputed) {
                    nx = Ac.size(1);
                    b_Ac.set_size(1, Ac.size(1));
                    for (i1 = 0; i1 < nx; i1++) {
                        b_Ac[i1] = Ac[b_i + Ac.size(0) * i1] * x[i1];
                    }
                    nx = b_Ac.size(1);
                    b_r.set_size(1, static_cast<int>(static_cast<unsigned short>(b_Ac.size(1))));
                    for (k = 0; k < nx; k++) {
                        b_r[k] = std::abs(b_Ac[k]);
                    }
                    cTol[b_i] = std::fmax(cTol[b_i], internal::maximum(b_r));
                }
                if (iA[b_i] == 0) {
                    nx = Ac.size(1);
                    c_Ac = 0.0;
                    for (i1 = 0; i1 < nx; i1++) {
                        c_Ac += Ac[b_i + Ac.size(0) * i1] * x[i1];
                    }
                    cVal = (c_Ac - b[b_i]) / cTol[b_i];
                    if (cVal < cMin) {
                        cMin = cVal;
                        kNext = static_cast<short>(b_i + 1);
                    }
                }
            }
            cTolComputed = true;
            if (kNext <= 0) {
                exitg2 = true;
            } else if (b_status == 200) {
                b_status = 0;
                exitg2 = true;
            } else {
                int exitg1;
                do {
                    exitg1 = 0;
                    if ((kNext > 0) && (b_status <= 200)) {
                        boolean_T guard2{false};
                        nx = Ac.size(1);
                        AcRow.set_size(1, Ac.size(1));
                        for (i = 0; i < nx; i++) {
                            AcRow[i] = Ac[(kNext + Ac.size(0) * i) - 1];
                        }
                        guard2 = false;
                        if (nA == 0) {
                            nx = Ac.size(1);
                            d_Ac.set_size(1, Ac.size(1));
                            for (i = 0; i < nx; i++) {
                                d_Ac[i] = Ac[(kNext + Ac.size(0) * i) - 1];
                            }
                            internal::blas::mtimes(Hinv, d_Ac, Rhs);
                            guard2 = true;
                        } else {
                            cVal = KWIKfactor(Ac, iC_data, nA, Linv, RLinv, D, H, n);
                            if (cVal <= 0.0) {
                                b_status = -2;
                                exitg1 = 1;
                            } else {
                                U.set_size(H.size(0), H.size(1));
                                nx = H.size(0) * H.size(1);
                                for (i = 0; i < nx; i++) {
                                    U[i] = -H[i];
                                }
                                nx = Ac.size(1);
                                d_Ac.set_size(1, Ac.size(1));
                                for (i = 0; i < nx; i++) {
                                    d_Ac[i] = Ac[(kNext + Ac.size(0) * i) - 1];
                                }
                                nx = U.size(0) - 1;
                                inner = U.size(1);
                                Rhs.set_size(U.size(0));
                                for (b_i = 0; b_i <= nx; b_i++) {
                                    Rhs[b_i] = 0.0;
                                }
                                for (k = 0; k < inner; k++) {
                                    aoffset = k * U.size(0);
                                    for (b_i = 0; b_i <= nx; b_i++) {
                                        Rhs[b_i] = Rhs[b_i] + U[aoffset + b_i] * d_Ac[k];
                                    }
                                }
                                i = nA;
                                for (b_i = 0; b_i < i; b_i++) {
                                    cVal = 0.0;
                                    nx = AcRow.size(1);
                                    for (i1 = 0; i1 < nx; i1++) {
                                        cVal += AcRow[i1] * D[i1 + D.size(0) * b_i];
                                    }
                                    r[b_i] = cVal;
                                }
                                guard2 = true;
                            }
                        }
                        if (guard2) {
                            double t1;
                            boolean_T exitg4;
                            kDrop = 0;
                            t1 = 0.0;
                            ColdReset = true;
                            DualFeasible = true;
                            if (nA > 6) {
                                nx = 0;
                                exitg4 = false;
                                while ((!exitg4) && (nx <= nA - 7)) {
                                    if (r[nx] >= 1.0E-12) {
                                        DualFeasible = false;
                                        exitg4 = true;
                                    } else {
                                        nx++;
                                    }
                                }
                            }
                            if ((nA != 6) && (!DualFeasible)) {
                                i = nA - 6;
                                if (nA - 6 < -32768) {
                                    i = -32768;
                                }
                                i = static_cast<short>(i);
                                for (b_i = 0; b_i < i; b_i++) {
                                    cVal = r[b_i];
                                    if (cVal > 1.0E-12) {
                                        cVal = lambda[iC_data[b_i] - 1] / cVal;
                                        if ((kDrop == 0) || (cVal < rMin)) {
                                            rMin = cVal;
                                            kDrop = static_cast<short>(b_i + 1);
                                        }
                                    }
                                }
                                if (kDrop > 0) {
                                    t1 = rMin;
                                    ColdReset = false;
                                }
                            }
                            cVal = 0.0;
                            if (Rhs.size(0) >= 1) {
                                nx = Rhs.size(0);
                                for (k = 0; k < nx; k++) {
                                    cVal += Rhs[k] * AcRow[k];
                                }
                            }
                            if (cVal <= 0.0) {
                                cVal = 0.0;
                                DualFeasible = true;
                            } else {
                                nx = Ac.size(1);
                                c_Ac = 0.0;
                                for (i = 0; i < nx; i++) {
                                    c_Ac += Ac[(kNext + Ac.size(0) * i) - 1] * x[i];
                                }
                                cVal = (b[kNext - 1] - c_Ac) / cVal;
                                DualFeasible = false;
                            }
                            if (ColdReset && DualFeasible) {
                                b_status = -1;
                                exitg1 = 1;
                            } else {
                                if (DualFeasible) {
                                    cMin = t1;
                                } else if (ColdReset) {
                                    cMin = cVal;
                                } else {
                                    cMin = std::fmin(t1, cVal);
                                }
                                i = nA;
                                for (b_i = 0; b_i < i; b_i++) {
                                    inner = iC_data[b_i];
                                    lambda[inner - 1] = lambda[inner - 1] - cMin * r[b_i];
                                    i1 = m - 6;
                                    if (m - 6 < -32768) {
                                        i1 = -32768;
                                    }
                                    if ((inner <= static_cast<short>(i1)) && (lambda[inner - 1] < 0.0)) {
                                        lambda[inner - 1] = 0.0;
                                    }
                                }
                                lambda[kNext - 1] = lambda[kNext - 1] + cMin;
                                if (cMin == t1) {
                                    DropConstraint(kDrop, iA, &nA, iC_data);
                                }
                                if (!DualFeasible) {
                                    nx = x.size(0);
                                    for (i = 0; i < nx; i++) {
                                        x[i] = x[i] + cMin * Rhs[i];
                                    }
                                    if (cMin == cVal) {
                                        if (nA == n) {
                                            b_status = -1;
                                            exitg1 = 1;
                                        } else {
                                            i = nA + 1;
                                            if (nA + 1 > 32767) {
                                                i = 32767;
                                            }
                                            nA = static_cast<short>(i);
                                            iC_data[static_cast<short>(i) - 1] = kNext;
                                            c_i = static_cast<short>(i);
                                            exitg4 = false;
                                            while ((!exitg4) && (c_i > 1)) {
                                                short i3;
                                                kDrop = iC_data[c_i - 1];
                                                i3 = iC_data[c_i - 2];
                                                if (kDrop > i3) {
                                                    exitg4 = true;
                                                } else {
                                                    iC_data[c_i - 1] = i3;
                                                    iC_data[c_i - 2] = kDrop;
                                                    c_i = static_cast<short>(c_i - 1);
                                                }
                                            }
                                            iA[kNext - 1] = 1;
                                            kNext = 0;
                                            b_status++;
                                        }
                                    } else {
                                        b_status++;
                                    }
                                } else {
                                    b_status++;
                                }
                            }
                        }
                    } else {
                        cVal = b_norm(x);
                        if (std::abs(cVal - Xnorm0) > 0.001) {
                            Xnorm0 = cVal;
                            nx = b.size(0);
                            Rhs.set_size(b.size(0));
                            for (k = 0; k < nx; k++) {
                                Rhs[k] = std::abs(b[k]);
                            }
                            cTol.set_size(Rhs.size(0));
                            nx = Rhs.size(0);
                            for (k = 0; k < nx; k++) {
                                cTol[k] = std::fmax(Rhs[k], 1.0);
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
    *status = b_status;
}

}

// 
// File trailer for qpkwik.cpp 
//  
// [EOF] 
//
