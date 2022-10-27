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

enum class MotorName {
  Left  = 0,
  Right = 1,
  Z     = 2,
  Theta = 3,
};

enum class JointName {
  Theta = 0,
  Z     = 1,
  R     = 2,
  Phi   = 3,
};

enum class AirState {
  On  = 1,
  Off = 0
};

enum class HandState {
  Shrink = 0,
  Extend = 1
};

enum class MoveMode {
  Manual = 0,
  Auto   = 1,
  Stop   = 2
};

enum class ZAutoState {
  Approach = 0,
  Depart   = 1
};

}  // namespace kirin_types

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES */
