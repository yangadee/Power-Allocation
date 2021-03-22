//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fileManager.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

#ifndef FILEMANAGER_H
#define FILEMANAGER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>
#include <stdio.h>

// Function Declarations
namespace coder {
int cfclose(double fid);

signed char cfopen(const char *cfilename, const char *cpermission);

void getfilestar(double fid, FILE **filestar, boolean_T *autoflush);

} // namespace coder
void filedata_init();

#endif
//
// File trailer for fileManager.h
//
// [EOF]
//
