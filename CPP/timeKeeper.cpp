//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: timeKeeper.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "timeKeeper.h"
#include "Tau2Force.h"
#include "rt_nonfinite.h"
#include "coder_posix_time.h"

// Function Definitions
//
// Arguments    : Tau2Force *aInstancePtr
//                double *outTime_tv_sec
//                double *outTime_tv_nsec
// Return Type  : void
//
namespace coder
{
  namespace internal
  {
    namespace time
    {
      namespace impl
      {
        void timeKeeper(Tau2Force *aInstancePtr, double *outTime_tv_sec, double *
                        outTime_tv_nsec)
        {
          Fun_Local_Convex_QP_4CppStackData *localSD;
          localSD = aInstancePtr->getStackData();
          *outTime_tv_sec = localSD->pd->savedTime.tv_sec;
          *outTime_tv_nsec = localSD->pd->savedTime.tv_nsec;
        }

        //
        // Arguments    : Tau2Force *aInstancePtr
        //                double newTime_tv_sec
        //                double newTime_tv_nsec
        // Return Type  : void
        //
        void timeKeeper(Tau2Force *aInstancePtr, double newTime_tv_sec, double
                        newTime_tv_nsec)
        {
          Fun_Local_Convex_QP_4CppStackData *localSD;
          struct timespec b_timespec;
          localSD = aInstancePtr->getStackData();
          if (!localSD->pd->savedTime_not_empty) {
            clock_gettime(CLOCK_MONOTONIC, &b_timespec);
            localSD->pd->savedTime_not_empty = true;
          }

          localSD->pd->savedTime.tv_sec = newTime_tv_sec;
          localSD->pd->savedTime.tv_nsec = newTime_tv_nsec;
        }
      }
    }
  }
}

//
// File trailer for timeKeeper.cpp
//
// [EOF]
//
