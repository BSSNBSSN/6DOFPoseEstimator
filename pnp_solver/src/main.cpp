#include "pnp_solver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pnp_solver_node");
  PNPSolver pnp_solver;
  ros::spin();
  return 0;
}
