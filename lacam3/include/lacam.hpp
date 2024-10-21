#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"
#include "planner.hpp"
#include "post_processing.hpp"
#include "sipp.hpp"
#include "utils.hpp"

namespace lacam
{

  Solution solve(const Instance &ins, const int threshold,
                 const int verbose = 0, const Deadline *deadline = nullptr,
                 const int seed = 0);

}  // namespace lacam
