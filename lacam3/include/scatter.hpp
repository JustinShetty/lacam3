/*
 * Implementation of SUO
 *
 * references:
 * Optimizingspaceutilizationformoreeffective multi-robot path planning.
 * Shuai D Han and Jingjin Yu.
 * In Proceedings of IEEE International Conference on Robotics and Automation
 * (ICRA). 2022.
 */
#pragma once

#include "collision_table.hpp"
#include "dist_table.hpp"
#include "graph.hpp"
#include "utils.hpp"

namespace lacam
{

  struct Scatter {
    const Instance *ins;
    const Deadline *deadline;
    std::mt19937 MT;
    const int verbose;
    const int N;
    const int V_size;
    const int T;  // makespan lower bound
    DistTableMultiGoal *D;
    const int cost_margin;
    int sum_of_path_length;

    // outcome
    std::vector<Path> paths;
    // agent, label, vertex-id, next vertex
    std::vector<std::vector<std::unordered_map<int, Vertex *>>> scatter_data_labeled;

    // collision data
    CollisionTable CT;

    void construct();

    Scatter(const Instance *_ins, DistTableMultiGoal *_D,
            const Deadline *_deadline, const int seed = 0, int _verbose = 0,
            int _cost_margin = 2);

    void write_solution(const std::string &fileName) const;
  };

}  // namespace lacam
