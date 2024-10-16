/*
 * distance table with lazy evaluation, using BFS
 */
#pragma once

#include "graph.hpp"
#include "instance.hpp"
#include "utils.hpp"

struct DistTableMultiGoal {
  const int K;  // number of vertices
  std::vector<std::vector<std::vector<int>>>
      table;  // distance table, index: agent-id, goal-index, vertex-id

  int get(const int i, const int goal_index,
          const int v_id) const;  // agent, goal-index, vertex-id
  inline int get(const int i, const int goal_index,
                 const Vertex *v) const // agent, goal-index, vertex
  {
    return get(i, goal_index, v->id);
  }

  DistTableMultiGoal(const Instance *ins);
  DistTableMultiGoal(const Instance &ins) : DistTableMultiGoal(&ins) {}

  void setup(const Instance *ins);  // initialization
};
