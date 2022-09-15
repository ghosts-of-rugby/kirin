#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE

#include <math.h>
#include "ik_r.h"
#include "ik_phi.h"
#include "ik_theta.h"
#include "fk_x.h"
#include "fk_y.h"
#include "fk_psi.h"
#include "dr.h"
#include "dtheta.h"
#include "dphi.h"

// TODO: obtain mathine params from URDF files
namespace machine {
/* phi rotation radius */
constexpr double kPhiRadius      = 0.03979;
/* motor rotation radius */
constexpr double kMotorRadius    = 0.023875;
/* theta gear ratio */
constexpr double kThetaGearRatio = 25. / 136.;
// /* z gear ratio */
// constexpr double kZGearRatio = 1.0; // (temp)
// /* motor gear radius */
// constexpr double kZGearRadius = 1.0; // (temp)
constexpr double kZRatio         = 0.02387 * M_PI / (2. * M_PI);

/* z offset from base axis to theta link */
constexpr double kZOffsetBaseToTheta     = 0.022;
/* z offset from theta axis to z link */
constexpr double kZOffsetThetaToZ        = 0.128;
/* z offset from z link to r link */
constexpr double kZOffsetZToR            = 0.0;
/* z offset from r link to Phi link */
constexpr double kZOffsetRToPhi          = -0.0235;
/* z offset from phi link to top bellows */
constexpr double kZOffsetPhiToTopBellows = -0.042;
/* r offset from center axis to root of r link */
constexpr double kROffsetCenterToRRoot   = 0.201;
/* r offset from r root to phi link */
constexpr double kROffsetRRootToPhi      = 0.345;
/* offset from phi link to top bellows */
constexpr double kROffsetPhiToTopBellows = 0.05;

/* z offset displacement from z lowest to initial */
constexpr double kZOffsetInitialDisplacement = 0.1187;

/* initial top bellows position */
/* top bellows point is in  */
constexpr double kRedInitialPosX = 0.0001;
constexpr double kRedInitialPosY
    = -(kROffsetCenterToRRoot + kROffsetRRootToPhi + kROffsetPhiToTopBellows);
constexpr double kRedInitialPosZ = kZOffsetBaseToTheta + kZOffsetThetaToZ + kZOffsetZToR
                                   + kZOffsetRToPhi + kZOffsetPhiToTopBellows
                                   + kZOffsetInitialDisplacement;
constexpr double kRedInitialPsi = -M_PI / 2;

/* joint state limit */
namespace limit {
/* z joint limit */
constexpr double kZLower = 0.0;
constexpr double kZUpper = 0.15;
/* r joint limit */
constexpr double kRLower = 0.0;
constexpr double kRUpper = 0.413;
}  // namespace limit

/* bellows position parameter */
namespace bellows {
constexpr double kZ        = -0.042;
constexpr double kTop_X    = 0.05;
constexpr double kShrink_D = 0.10;
constexpr double kExtend_D = 0.14;
}  // namespace bellows

}  // namespace machine

namespace model {
/* model calculation */
double CalcR(double l, double x, double y, double psi);
double CalcPhi(double l, double x, double y, double psi);
double CalcTheta(double l, double x, double y, double psi);
double CalcX(double theta, double r, double phi, double l);
double CalcY(double theta, double r, double phi, double l);
double CalcPsi(double theta, double phi);
double CalcRVel(double l, double dx, double dy, double dpsi, double r, double theta, double phi);
double CalcPhiVel(double l, double dx, double dy, double dpsi, double r, double theta, double phi);
double CalcThetaVel(
    double l, double dx, double dy, double dpsi, double r, double theta, double phi);
}  // namespace model

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE */
