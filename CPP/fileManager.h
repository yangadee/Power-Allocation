//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fileManager.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//
#ifndef FILEMANAGER_H
#define FILEMANAGER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <stdio.h>

// Type Declarations
class Tau2Force;

// Function Declarations
namespace coder
{
  int cfclose(Tau2Force *aInstancePtr, double fid);
  signed char cfopen(Tau2Force *aInstancePtr, const char *cfilename, const char *
                     cpermission);
  void getfilestar(Tau2Force *aInstancePtr, double fid, FILE * *filestar, bool
                   *autoflush);
}

#endif

//
// File trailer for fileManager.h
//
// [EOF]
//
