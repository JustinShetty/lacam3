/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

namespace lacam
{

  struct Instance {
    Graph *G;       // graph
    Config starts;  // initial configuration
    std::vector<std::vector<Vertex *>>
        goal_sequences;  // agent-id -> goal sequence
    const uint N;        // number of agents
    bool delete_graph_after_used;

    Instance(Graph *_G, const Config &_starts,
             const std::vector<std::vector<Vertex *>> &_goal_sequences,
             uint _N);
    Instance(const std::string &map_filename,
             const std::vector<int> &start_indexes,
             const std::vector<int> &goal_indexes);
    Instance(const std::string &map_filename,
             const std::vector<int> &start_indexes,
             const std::vector<std::vector<int>> &goal_index_sequences);
    // for MAPF benchmark
    Instance(const std::string &scen_filename, const std::string &map_filename,
             const int _N = 1);
    // random instance generation
    Instance(const std::string &map_filename, const int _N = 1,
             const int seed = 0);
    ~Instance();

    // simple feasibility check of instance
    bool is_valid(const int verbose = 0) const;

    int get_total_goals() const;

    bool is_goal_config(const Config &C) const;
  };

  // solution: a sequence of configurations
  using Solution = std::vector<Config>;

}  // namespace lacam
