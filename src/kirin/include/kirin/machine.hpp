#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE

#include <math.h>

// TODO: obtain mathine params from URDF file
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
constexpr double kZRatio         = 0.02387 / 2. * M_PI;

/* z offset from base axis to theta link */
constexpr double kZOffsetBaseToTheta  = 0.022;
/* z offset from theta axis to z link */
constexpr double kZOffsetThetaToZ     = 0.128;
/* r offset from center axis to root of r link */
constexpr double kOffsetCenterToRRoot = 0.201;
/* r offset from r root to phi link */
constexpr double kOffsetRRootToPhi    = 0.345;

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

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE */
