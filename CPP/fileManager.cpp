//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fileManager.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

// Include Files
#include "fileManager.h"
#include "Tau2Force.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <stdio.h>

// Function Declarations
namespace coder
{
  static signed char filedata(Tau2Force *aInstancePtr);
}

// Function Definitions
//
// Arguments    : Tau2Force *aInstancePtr
// Return Type  : signed char
//
namespace coder
{
  static signed char filedata(Tau2Force *aInstancePtr)
  {
    Fun_Local_Convex_QP_4CppStackData *localSD;
    int k;
    signed char f;
    bool exitg1;
    localSD = aInstancePtr->getStackData();
    f = 0;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < 20)) {
      if (localSD->pd->eml_openfiles[k] == NULL) {
        f = static_cast<signed char>(k + 1);
        exitg1 = true;
      } else {
        k++;
      }
    }

    return f;
  }

  //
  // Arguments    : Tau2Force *aInstancePtr
  //                double fid
  // Return Type  : int
  //
  int cfclose(Tau2Force *aInstancePtr, double fid)
  {
    FILE * filestar;
    Fun_Local_Convex_QP_4CppStackData *localSD;
    int st;
    signed char b_fileid;
    signed char fileid;
    localSD = aInstancePtr->getStackData();
    st = -1;
    fileid = static_cast<signed char>(std::round(fid));
    if ((fileid < 0) || (fid != fileid)) {
      fileid = -1;
    }

    b_fileid = fileid;
    if (fileid < 0) {
      b_fileid = -1;
    }

    if (b_fileid >= 3) {
      filestar = localSD->pd->eml_openfiles[b_fileid - 3];
    } else {
      switch (b_fileid) {
       case 0:
        filestar = stdin;
        break;

       case 1:
        filestar = stdout;
        break;

       case 2:
        filestar = stderr;
        break;

       default:
        filestar = NULL;
        break;
      }
    }

    if ((filestar != NULL) && (fileid >= 3)) {
      int cst;
      cst = fclose(filestar);
      if (cst == 0) {
        st = 0;
        localSD->pd->eml_openfiles[fileid - 3] = NULL;
        localSD->pd->eml_autoflush[fileid - 3] = true;
      }
    }

    return st;
  }

  //
  // Arguments    : Tau2Force *aInstancePtr
  //                const char *cfilename
  //                const char *cpermission
  // Return Type  : signed char
  //
  signed char cfopen(Tau2Force *aInstancePtr, const char *cfilename, const char *
                     cpermission)
  {
    Fun_Local_Convex_QP_4CppStackData *localSD;
    signed char fileid;
    signed char j;
    localSD = aInstancePtr->getStackData();
    fileid = -1;
    j = filedata(aInstancePtr);
    if (j >= 1) {
      FILE * filestar;
      filestar = fopen(cfilename, cpermission);
      if (filestar != NULL) {
        int i;
        localSD->pd->eml_openfiles[j - 1] = filestar;
        localSD->pd->eml_autoflush[j - 1] = true;
        i = j + 2;
        if (j + 2 > 127) {
          i = 127;
        }

        fileid = static_cast<signed char>(i);
      }
    }

    return fileid;
  }

  //
  // Arguments    : Tau2Force *aInstancePtr
  //                double fid
  //                FILE * *filestar
  //                bool *autoflush
  // Return Type  : void
  //
  void getfilestar(Tau2Force *aInstancePtr, double fid, FILE * *filestar, bool
                   *autoflush)
  {
    Fun_Local_Convex_QP_4CppStackData *localSD;
    signed char fileid;
    localSD = aInstancePtr->getStackData();
    fileid = static_cast<signed char>(std::round(fid));
    if ((fileid < 0) || (fid != fileid)) {
      fileid = -1;
    }

    if (fileid >= 3) {
      *filestar = localSD->pd->eml_openfiles[fileid - 3];
      *autoflush = localSD->pd->eml_autoflush[fileid - 3];
    } else {
      switch (fileid) {
       case 0:
        *filestar = stdin;
        *autoflush = true;
        break;

       case 1:
        *filestar = stdout;
        *autoflush = true;
        break;

       case 2:
        *filestar = stderr;
        *autoflush = true;
        break;

       default:
        *filestar = NULL;
        *autoflush = true;
        break;
      }
    }
  }
}

//
// File trailer for fileManager.cpp
//
// [EOF]
//
