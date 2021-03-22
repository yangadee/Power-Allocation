//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: timeKeeper.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//
#ifndef TIMEKEEPER_H
#define TIMEKEEPER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
class Tau2Force;

// Function Declarations
namespace coder
{
  namespace internal
  {
    namespace time
    {
      namespace impl
      {
        void timeKeeper(Tau2Force *aInstancePtr, double *outTime_tv_sec, double *
                        outTime_tv_nsec);
        void timeKeeper(Tau2Force *aInstancePtr, double newTime_tv_sec, double
                        newTime_tv_nsec);
      }
    }
  }
}

#endif

//
// File trailer for timeKeeper.h
//
// [EOF]
//
