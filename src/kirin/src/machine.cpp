
#include "kirin/machine.hpp"

namespace model {
constexpr int kIkIndex = 1;

double CalcR(double l, double x, double y, double psi) {
  double out[2];
  model::ik_r(l, psi, x, y, out);
  return out[kIkIndex];
}

double CalcPhi(double l, double x, double y, double psi) {
  if(std::abs(std::atan2(y, x) - psi) <= 0.0001) return 0.0;
  double out[2];
  model::ik_phi(l, psi, x, y, out);
  return isnan(out[kIkIndex]) ? 0.0 : out[kIkIndex];
}

double CalcTheta(double l, double x, double y, double psi) {
  if(std::abs(std::atan2(y, x) - psi) <= 0.0001) return 0.0;
  double out[2];
  model::ik_theta(l, psi, x, y, out);
  return isnan(out[kIkIndex]) ? 0.0 : out[kIkIndex];
}

double CalcX(double theta, double r, double phi, double l) { return model::fk_x(l, phi, r, theta); }

double CalcY(double theta, double r, double phi, double l) { return model::fk_y(l, phi, r, theta); }

double CalcPsi(double theta, double phi) { return model::fk_psi(phi, theta); }

double CalcRVel(double l, double dx, double dy, double dpsi, double r, double theta, double phi) {
  return model::dr(dpsi, dx, dy, l, phi, theta);
}
double CalcPhiVel(double l, double dx, double dy, double dpsi, double r, double theta, double phi) {
  return model::dphi(dpsi, dx, dy, l, phi, r, theta);
}
double CalcThetaVel(
    double l, double dx, double dy, double dpsi, double r, double theta, double phi) {
  return model::dtheta(dpsi, dx, dy, l, phi, r, theta);
}

}  // namespace model