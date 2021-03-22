//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_Test_Actuator_Mod_V4_api.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 14-Mar-2021 05:17:40
//

// Include Files
#include "_coder_Test_Actuator_Mod_V4_api.h"
#include "_coder_Test_Actuator_Mod_V4_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131610U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "Test_Actuator_Mod_V4",                               // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void Test_Actuator_Mod_V4_api()
{
  // Invoke the target function
  Test_Actuator_Mod_V4();
}

//
// Arguments    : void
// Return Type  : void
//
void Test_Actuator_Mod_V4_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  Test_Actuator_Mod_V4_xil_terminate();
  Test_Actuator_Mod_V4_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void Test_Actuator_Mod_V4_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

//
// Arguments    : void
// Return Type  : void
//
void Test_Actuator_Mod_V4_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

//
// File trailer for _coder_Test_Actuator_Mod_V4_api.cpp
//
// [EOF]
//
