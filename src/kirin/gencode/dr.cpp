//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// dr.cpp
//
// Code generation for function 'dr'
//

// Include files
#include "dr.h"
#include <cmath>

// Function Definitions
namespace model {
double dr(double dp0, double dx0, double dy0, double l, double phi, double th)
{
  // DR
  //     OUT1 = DR(DP0,DX0,DY0,L,PHI,TH)
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     09-Sep-2022 17:49:00
  return (dx0 * std::cos(th) + dy0 * std::sin(th)) + dp0 * l * std::sin(phi);
}

} // namespace model

// End of code generation (dr.cpp)
