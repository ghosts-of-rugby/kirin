#include "kirin/optimized_path_generator.hpp"

using Sl = casadi::Slice;

OptimizedPathGenerator::OptimizedPathGenerator(const rclcpp::NodeOptions& options)
    : rclcpp::Node("optimized_path_generator", options) {}

OptimizedPathGenerator::~OptimizedPathGenerator() {}

DirectCollocationSolver::DirectCollocationSolver(): solver_state(SolverState::NotInitialized) {}

DirectCollocationSolver::~DirectCollocationSolver() {}

ca::Function DirectCollocationSolver::GetForwardKinematics(double l) {
  ca::MX r   = ca::MX::sym("r", 1);
  ca::MX th  = ca::MX::sym("th", 1);
  ca::MX phi = ca::MX::sym("phi", 1);

  // forward kinematics
  auto psi = th + phi;
  auto x   = r * cos(th) + l * cos(psi);
  auto y   = r * sin(th) + l * sin(psi);

  return ca::Function("fk", {r, th, phi}, {x, y, psi});
}

ca::Function DirectCollocationSolver::GetIntegratorDynamics() {
  ca::MX x  = ca::MX::sym("x");
  ca::MX v  = ca::MX::sym("v");
  ca::MX a  = ca::MX::sym("a");
  ca::MX dt = ca::MX::sym("dt");

  return ca::Function("integrator", {x, v, a, dt}, {x + dt * (v + 0.5 * dt * a), v + dt * a});
}

void DirectCollocationSolver::SetupOpti(const ProblemSettings& settings) {
  // function
  forward_kinematics = GetForwardKinematics(settings.model_params.l);
  integrator         = GetIntegratorDynamics();

  // length
  casadi_int phase_length = static_cast<casadi_int>(settings.phase_length);
  casadi_int N = 2 * phase_length;
  npoints = N+1;

  // variable settings
  x       = opti.variable(1, N + 1);
  y       = opti.variable(1, N + 1);
  psi     = opti.variable(1, N + 1);
  r       = opti.variable(1, N + 1);
  theta   = opti.variable(1, N + 1);
  phi     = opti.variable(1, N + 1);
  dr      = opti.variable(1, N + 1);
  dtheta  = opti.variable(1, N + 1);
  dphi    = opti.variable(1, N + 1);
  ddr     = opti.variable(1, N + 1);
  ddtheta = opti.variable(1, N + 1);
  ddphi   = opti.variable(1, N + 1);
  dt      = opti.variable();

  // constraint
  opti.subject_to(x(0) == settings.start(0));
  opti.subject_to(y(0) == settings.start(1));
  opti.subject_to(psi(0) == settings.start(2));
  opti.subject_to(x(N) == settings.end(0));
  opti.subject_to(y(N) == settings.end(1));
  opti.subject_to(psi(N) == settings.end(2));
  opti.subject_to(dr(0) == ca::MX::zeros());
  opti.subject_to(ddr(0) == ca::MX::zeros());
  opti.subject_to(dtheta(0) == ca::MX::zeros());
  opti.subject_to(ddtheta(0) == ca::MX::zeros());
  opti.subject_to(dphi(0) == ca::MX::zeros());
  opti.subject_to(ddphi(0) == ca::MX::zeros());
  opti.subject_to(dr(N) == ca::MX::zeros());
  opti.subject_to(ddr(N) == ca::MX::zeros());
  opti.subject_to(dtheta(N) == ca::MX::zeros());
  opti.subject_to(ddtheta(N) == ca::MX::zeros());
  opti.subject_to(dphi(N) == ca::MX::zeros());
  opti.subject_to(ddphi(N) == ca::MX::zeros());

  double w          = settings.cost_weights;
  Limit r_limit     = settings.model_params.r_limit;
  Limit theta_limit = settings.model_params.theta_limit;
  Limit phi_limit   = settings.model_params.phi_limit;

  for (casadi_int k = 0; k < N; ++k) {
    // samples: x(k), x(k+1), x(k+2),,,,,,

    // integrator constraint
    opti.subject_to(ca::MX::vertcat({r(Sl(), k + 1), dr(Sl(), k + 1)})
                    == ca::MX::vertcat(integrator({r(Sl(), k), dr(Sl(), k), ddr(Sl(), k), dt})));
    opti.subject_to(
        ca::MX::vertcat({theta(Sl(), k + 1), dtheta(Sl(), k + 1)})
        == ca::MX::vertcat(integrator({theta(Sl(), k), dtheta(Sl(), k), ddtheta(Sl(), k), dt})));
    opti.subject_to(
        ca::MX::vertcat({phi(Sl(), k + 1), dphi(Sl(), k + 1)})
        == ca::MX::vertcat(integrator({phi(Sl(), k), dphi(Sl(), k), ddphi(Sl(), k), dt})));

    // kinematics constraint
    opti.subject_to(
        ca::MX::vertcat({x(Sl(), k), y(Sl(), k), psi(Sl(), k)})
        == ca::MX::vertcat(forward_kinematics({r(Sl(), k), theta(Sl(), k), phi(Sl(), k)})));

    // limit constraint
    opti.subject_to(r_limit.min <= r(k) <= r_limit.max);
    opti.subject_to(-r_limit.max_speed <= dr(k) <= r_limit.max_speed);
    opti.subject_to(-r_limit.max_accel <= ddr(k) <= r_limit.max_accel);
    opti.subject_to(theta_limit.min <= theta(k) <= theta_limit.max);
    opti.subject_to(-theta_limit.max_speed <= dtheta(k) <= theta_limit.max_speed);
    opti.subject_to(-theta_limit.max_accel <= ddtheta(k) <= theta_limit.max_accel);
    opti.subject_to(phi_limit.min <= phi(k) <= phi_limit.max);
    opti.subject_to(-phi_limit.max_speed <= dphi(k) <= phi_limit.max_speed);
    opti.subject_to(-phi_limit.max_accel <= ddphi(k) <= phi_limit.max_accel);
  }
  // kinematics constraint
  opti.subject_to(
      ca::MX::vertcat({x(Sl(), N), y(Sl(), N), psi(Sl(), N)})
      == ca::MX::vertcat(forward_kinematics({r(Sl(), N), theta(Sl(), N), phi(Sl(), N)})));

  // limit constraint
  opti.subject_to(r_limit.min <= r(0, N) <= r_limit.max);
  opti.subject_to(-r_limit.max_speed <= dr(0, N) <= r_limit.max_speed);
  opti.subject_to(-r_limit.max_accel <= ddr(0, N) <= r_limit.max_accel);
  opti.subject_to(theta_limit.min <= theta(0, N) <= theta_limit.max);
  opti.subject_to(-theta_limit.max_speed <= dtheta(0, N) <= theta_limit.max_speed);
  opti.subject_to(-theta_limit.max_accel <= ddtheta(0, N) <= theta_limit.max_accel);
  opti.subject_to(phi_limit.min <= phi(0, N) <= phi_limit.max);
  opti.subject_to(-phi_limit.max_speed <= dphi(0, N) <= phi_limit.max_speed);
  opti.subject_to(-phi_limit.max_accel <= ddphi(0, N) <= phi_limit.max_accel);

  // cost function
  ca::MX cost_function = (N + 1) * dt;
  opti.minimize(cost_function);
}

bool DirectCollocationSolver::SetupProblem(const ProblemSettings& settings) {
  // setup opti
  SetupOpti(settings);

  casadi::Dict casadi_options;
  casadi::Dict ipopt_options;

  casadi_options["expand"] = true;//Replace MX with SX expressions in problem formulation, speed up
  unsigned long solver_verbosity = settings.solver_verbosity;
  if (solver_verbosity) {
      casadi_int ipoptVerbosity = static_cast<long long>(solver_verbosity - 1);
      ipopt_options["print_level"] = ipoptVerbosity;
      casadi_options["print_time"] = true;
      casadi_options["bound_consistency"] = false;
  } else {
      ipopt_options["print_level"] = 0;
      casadi_options["print_time"] = false;
      //casadiOptions["bound_consistency"] = false;
      //ipoptOptions["fixed_variable_treatment"] = "make_constraint";
  }
  ipopt_options["linear_solver"] = settings.ipopt_linear_solver;

  opti.solver("ipopt", casadi_options, ipopt_options);

  solver_state = SolverState::ProblemSet;

  return true;

}

bool DirectCollocationSolver::Solve() {
  if (solver_state == SolverState::NotInitialized) {
    throw std::runtime_error("problem is not initialized");
    return false;
  }

  // set initial
  

  // try to solve
  try {
    solution = std::make_unique<ca::OptiSol>(opti.solve());
  } catch (std::exception& e) {
    opti.debug().show_infeasibilities(1e-5);
    std::cerr << "error while solving the optimization" << std::endl;
    std::cerr << "Details:\n " << e.what() << std::endl;
    return false;
  }
  solver_state = SolverState::ProblemSolved;
  std::cout << "\n solve success \n\n";
  return true;
}