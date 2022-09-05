#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE

// TODO: obtain mathine params from URDF file
namespace machine {
  // TODO: make temp true parameter
  /* phi rotation radius */
  constexpr double kPhiRadius = 0.1; // (temp)
  /* motor rotation radius */
  constexpr double kMotorRadius = 0.5; // (temp)
  /* theta gear ratio */
  constexpr double kThetaGearRatio = 1.0; // (temp)
  /* z gear ratio */
  constexpr double kZGearRatio = 1.0; // (temp)
  /* motor gear radius */
  constexpr double kZGearRadius = 1.0; // (temp)


  /* z offset from base axis to theta link */
  constexpr double kZOffsetBaseToTheta = 0.022;
  /* z offset from theta axis to z link */
  constexpr double kZOffsetThetaToZ = 0.128;
  /* r offset from center axis to root of r link */
  constexpr double kOffsetCenterToRRoot = 0.201;
  /* r offset from r root to phi link */
  constexpr double kOffsetRRootToPhi = 0.345;


  /* joint state limit */
  namespace limit {
    /* z joint limit */
    constexpr double kZLower = 0.0;
    constexpr double kZUpper = 0.15;
    /* r joint limit */
    constexpr double kRLower = 0.0;
    constexpr double kRUpper = 0.413;
  }
}



#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_MACHINE */
