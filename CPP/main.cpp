//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 03-Feb-2021 11:24:09
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include Files
#include "main.h"
#include "Tau2Force.h"
#include "rt_nonfinite.h"

// Variable Definitions
static Fun_Local_Convex_QP_4CppStackData Fun_Local_Convex_QP_4CppStackDataGlobal;
static Fun_Local_Convex_QP_4CppPersistentData
  Fun_Local_Convex_QP_4CppPersistentDataGlobal;

// Function Declarations
static void main_Fun_Local_Convex_QP_4Cpp(Tau2Force *instancePtr);

// Function Definitions
//
// Arguments    : Tau2Force *instancePtr
// Return Type  : void
//
static void main_Fun_Local_Convex_QP_4Cpp(Tau2Force *instancePtr)
{
  // Call the entry-point 'Fun_Local_Convex_QP_4Cpp'.
  instancePtr->Fun_Local_Convex_QP_4Cpp();
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  Tau2Force *classInstance;
  classInstance = new Tau2Force;
  Fun_Local_Convex_QP_4CppStackDataGlobal.pd =
    &Fun_Local_Convex_QP_4CppPersistentDataGlobal;

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_Fun_Local_Convex_QP_4Cpp(classInstance);
  delete classInstance;
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
