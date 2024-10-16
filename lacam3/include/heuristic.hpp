/*
 * heuristic definition
 */

#pragma once
#include "dist_table.hpp"
#include "graph.hpp"
#include "instance.hpp"

struct Heuristic {
  const Instance *ins;
  DistTableMultiGoal *D;

  Heuristic(const Instance *_ins, DistTableMultiGoal *_D);
  int get(const Config &C) const;
};
