#ifndef SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_OPTIMIZED_PATH_GENERATOR
#define SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_OPTIMIZED_PATH_GENERATOR

#include <utility>
#include <array>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>

namespace ca = casadi;

class OptimizedPathGenerator : public rclcpp::Node {
 public:
  OptimizedPathGenerator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~OptimizedPathGenerator();

};

struct Limit {
  double min;
  double max;
  double max_speed;
  double max_accel;
};

struct ModelParameters {
  double l;
  Limit r_limit;
  Limit theta_limit;
  Limit phi_limit;
};

struct ProblemSettings {
  int phase_length;  // size of collocation variables
  ModelParameters model_params;
  ca::DM start;
  ca::DM end;
  double cost_weights;
  double solver_verbosity;
  std::string ipopt_linear_solver;
};

class DirectCollocationSolver {
 public:

  enum class SolverState {
    NotInitialized,
    ProblemSet,
    ProblemSolved
  };
  
  DirectCollocationSolver(); 
  ~DirectCollocationSolver();

  bool SetupProblem(const ProblemSettings& settings);
  bool Solve();

 private:
  void SetupOpti(const ProblemSettings& settings);
  ca::Function GetForwardKinematics(double l);
  ca::Function GetIntegratorDynamics();
  ca::Function forward_kinematics;
  ca::Function integrator;
  SolverState solver_state;

  ca::MX x, y, psi;
  ca::MX r, theta, phi;
  ca::MX dr, dtheta, dphi;
  ca::MX ddr, ddtheta, ddphi;  // input
  ca::MX dt;
  ca::Opti opti;
  ModelParameters model_params;
  casadi_int npoints;

  std::unique_ptr<ca::OptiSol> solution;
};

#endif /* SRC_CATCHROBO_SRC_KIRIN_INCLUDE_KIRIN_OPTIMIZED_PATH_GENERATOR */
