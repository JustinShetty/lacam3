#include "../include/heuristic.hpp"

namespace lacam
{

  Heuristic::Heuristic(const Instance *_ins, DistTableMultiGoal *_D)
      : ins(_ins), D(_D)
  {
  }

  int Heuristic::get(const Config &Q) const
  {
    auto cost = 0;
    for (size_t i = 0; i < ins->N; ++i)
      cost += D->get(i, Q.goal_indices[i], Q[i]);
    return cost;
  }

}  // namespace lacam
