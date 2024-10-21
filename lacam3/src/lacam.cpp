#include "../include/lacam.hpp"

namespace lacam {

Solution solve(const Instance &ins, const int threshold, int verbose,
               const Deadline *deadline, const int seed)
{
  info(1, verbose, deadline, "pre-processing");
  auto planner = Planner(&ins, threshold, verbose, deadline, seed);
  return planner.solve();
}

} // namespace lacam
