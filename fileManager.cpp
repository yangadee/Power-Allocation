//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fileManager.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "fileManager.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <stdio.h>
#include <string.h>

// Variable Definitions
static FILE *eml_openfiles[20];

static boolean_T eml_autoflush[20];

// Function Declarations
namespace coder {
static signed char filedata();

}

// Function Definitions
//
// Arguments    : void
// Return Type  : signed char
//
namespace coder {
static signed char filedata()
{
  int k;
  signed char f;
  boolean_T exitg1;
  f = 0;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 20)) {
    if (eml_openfiles[k] == NULL) {
      f = static_cast<signed char>(k + 1);
      exitg1 = true;
    } else {
      k++;
    }
  }
  return f;
}

//
// Arguments    : double fid
// Return Type  : int
//
int cfclose(double fid)
{
  FILE *filestar;
  int st;
  signed char b_fileid;
  signed char fileid;
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
    filestar = eml_openfiles[b_fileid - 3];
  } else if (b_fileid == 0) {
    filestar = stdin;
  } else if (b_fileid == 1) {
    filestar = stdout;
  } else if (b_fileid == 2) {
    filestar = stderr;
  } else {
    filestar = NULL;
  }
  if ((filestar != NULL) && (fileid >= 3)) {
    int cst;
    cst = fclose(filestar);
    if (cst == 0) {
      st = 0;
      eml_openfiles[fileid - 3] = NULL;
      eml_autoflush[fileid - 3] = true;
    }
  }
  return st;
}

//
// Arguments    : const char *cfilename
//                const char *cpermission
// Return Type  : signed char
//
signed char cfopen(const char *cfilename, const char *cpermission)
{
  signed char fileid;
  signed char j;
  fileid = -1;
  j = filedata();
  if (j >= 1) {
    FILE *filestar;
    filestar = fopen(cfilename, cpermission);
    if (filestar != NULL) {
      int i;
      eml_openfiles[j - 1] = filestar;
      eml_autoflush[j - 1] = true;
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
// Arguments    : double fid
//                FILE * *filestar
//                boolean_T *autoflush
// Return Type  : void
//
void getfilestar(double fid, FILE **filestar, boolean_T *autoflush)
{
  signed char fileid;
  fileid = static_cast<signed char>(std::round(fid));
  if ((fileid < 0) || (fid != fileid)) {
    fileid = -1;
  }
  if (fileid >= 3) {
    *filestar = eml_openfiles[fileid - 3];
    *autoflush = eml_autoflush[fileid - 3];
  } else if (fileid == 0) {
    *filestar = stdin;
    *autoflush = true;
  } else if (fileid == 1) {
    *filestar = stdout;
    *autoflush = true;
  } else if (fileid == 2) {
    *filestar = stderr;
    *autoflush = true;
  } else {
    *filestar = NULL;
    *autoflush = true;
  }
}

//
// Arguments    : void
// Return Type  : void
//
} // namespace coder
void filedata_init()
{
  FILE *a;
  a = NULL;
  for (int i{0}; i < 20; i++) {
    eml_autoflush[i] = false;
    eml_openfiles[i] = a;
  }
}

//
// File trailer for fileManager.cpp
//
// [EOF]
//
