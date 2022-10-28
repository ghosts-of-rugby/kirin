#include <kirin/optimized_path_generator.hpp>


int main() {
  ModelParameters model_params{
    .l = 0.05,
    .r_limit = {.min = 0.3, .max = 1.0, .max_speed = 1.0, .max_accel = 5.0},
    .theta_limit = {.min = -M_PI, .max = M_PI, .max_speed = M_PI*2, .max_accel = M_PI*5},
    .phi_limit = {.min = -M_PI, .max = M_PI, .max_speed = M_PI*2, .max_accel = M_PI*5}
  };

  ProblemSettings settings;
  settings.phase_length = 100;
  settings.model_params = model_params;
  settings.start = { 0.5, 0.0, 0.0 };
  settings.end = { 0.0, 0.3, M_PI_2 };
  settings.cost_weights = 1;
  settings.solver_verbosity = 1;
  settings.ipopt_linear_solver = "mumps";

  DirectCollocationSolver solver;
  solver.SetupProblem(settings);
  solver.Solve();
}