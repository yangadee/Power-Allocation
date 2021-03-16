//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: qpkwik.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

#ifndef QPKWIK_H
#define QPKWIK_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void qpkwik(const ::coder::array<double, 2U> &Linv,
            const ::coder::array<double, 2U> &Hinv,
            const ::coder::array<double, 1U> &f,
            const ::coder::array<double, 2U> &Ac,
            const ::coder::array<double, 1U> &b, ::coder::array<short, 1U> &iA,
            short m, short n, ::coder::array<double, 1U> &x,
            ::coder::array<double, 1U> &lambda, double *status);

}

#endif
//
// File trailer for qpkwik.h
//
// [EOF]
//
