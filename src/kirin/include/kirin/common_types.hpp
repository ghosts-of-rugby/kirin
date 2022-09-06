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

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_COMMON_TYPES */
