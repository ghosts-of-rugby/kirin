
#include <vector>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>

namespace ca = casadi;

int main() {
  ca::MX x = ca::MX::sym("x");
  ca::MX y = ca::MX::sym("y");
  ca::MX z = ca::MX::sym("z");
  ca::MX f = pow(x,2)+100*pow(z,2.0);
  ca::MX g = z+pow(1-x,2)-y;

  ca::MXDict nlp;                 // NLP declaration
  nlp["x"] = vertcat(x,y,z);  // decision vars
  nlp["f"] = f;               // objective
  nlp["g"] = g;               // constraints

  // Create solver instance
  ca::Function F = ca::nlpsol("F","ipopt",nlp);

  // Solve the problem using a guess
  auto sol = F(ca::DMDict{{"x0",ca::DM({2.5,3.0,0.75})},{"ubg",0},{"lbg",0}});

  for (auto& [a,b] : sol) {
    std::cout << a << "  :  " << b << std::endl;
  }
}
