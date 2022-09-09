#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES

struct Joint {
  double theta;  // [rad]
  double z;      // [m]
  double r;      // [m]
  double phi;    // [rad]
};

struct MotorAngle {  // [rad]
  double theta;
  double left;
  double right;
  double z;
};

namespace kirin_types {

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

enum class HandState {
  Shrink = 0,
  Extend = 1
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES */
