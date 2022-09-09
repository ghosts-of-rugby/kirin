//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// dtheta.cpp
//
// Code generation for function 'dtheta'
//

// Include files
#include "dtheta.h"
#include <cmath>

// Function Definitions
namespace model {
double dtheta(double dp0, double dx0, double dy0, double l, double phi,
              double r, double th)
{
  // DTHETA
  //     OUT1 = DTHETA(DP0,DX0,DY0,L,PHI,R,TH)
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     09-Sep-2022 17:49:00
  return -((-dy0 * std::cos(th) + dx0 * std::sin(th)) +
           dp0 * l * std::cos(phi)) /
         r;
}

} // namespace model

// End of code generation (dtheta.cpp)
