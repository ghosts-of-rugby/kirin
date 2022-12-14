//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ik_phi.cpp
//
// Code generation for function 'ik_phi'
//

// Include files
#include "ik_phi.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace model {
void ik_phi(double l, double p0, double x0, double b_y0, double out1[2])
{
  double t11_tmp;
  double t20;
  double t6;
  double t7;
  double t9;
  // IK_PHI
  //     OUT1 = IK_PHI(L,P0,X0,Y0)
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     04-Sep-2022 21:54:38
  t6 = p0 / 2.0;
  t7 = std::cos(t6);
  t9 = t7 * t7;
  t11_tmp = l * t9;
  t6 = l * t7 * std::sin(t6);
  t20 = -1.0 / (t6 * 2.0 - b_y0);
  t7 = t9 * std::sqrt(1.0 / (t9 * t9) *
                      (((((l * l + x0 * x0) + b_y0 * b_y0) + l * x0 * 2.0) +
                        -(t11_tmp * x0 * 4.0)) +
                       -(t6 * b_y0 * 4.0)));
  t6 = l + -(t11_tmp * 2.0);
  out1[0] = p0 + std::atan(t20 * ((t6 + t7) + x0)) * 2.0;
  out1[1] = p0 + std::atan(t20 * ((t6 - t7) + x0)) * 2.0;
}

} // namespace model

// End of code generation (ik_phi.cpp)
