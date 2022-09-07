#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES



struct Joint {
  double theta; // [rad]
  double z;     // [m]
  double r;     // [m]
  double phi;   // [rad]
};

struct MotorAngle { // [rad]
  double theta;
  double left;
  double right;
  double z;
};

namespace kirin_type {
  enum class BellowsName {
    Top,
    Left,
    Right,
    ExLeft,
    ExRight
  };

  enum class AirState {
    On = 1,
    Off = 0
  };
}

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES */
