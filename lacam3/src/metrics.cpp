#include "../include/metrics.hpp"

namespace lacam {

int get_makespan(const Solution &solution)
{
  if (solution.empty()) return 0;
  return solution.size() - 1;
}

int get_makespan_paths(const std::vector<Path> &solution)
{
  auto c = 0;
  for (auto &&path : solution) {
    c = std::max(c, (int)path.size() - 1);
  }
  return c;
}

int get_path_cost(const Solution &solution, int i)
{
  const auto makespan = solution.size();
  const auto g = solution.back()[i];
  auto c = makespan;
  while (c > 0 && solution[c - 1][i] == g) --c;
  return c;
}

int get_path_cost(const Path &path)
{
  const auto g = path.back();
  auto c = path.size();
  while (c > 0 && path[c - 1] == g) --c;
  return c;
}

int get_sum_of_costs(const Solution &solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  for (size_t i = 0; i < N; ++i) c += get_path_cost(solution, i);
  return c;
}

int get_sum_of_costs_paths(const std::vector<Path> &solution)
{
  int c = 0;
  for (auto &&path : solution) c += get_path_cost(path);
  return c;
}

int get_path_loss(const Path &path)
{
  const auto g = path.back();
  const auto T = path.size();
  auto c = 0;
  for (size_t t = 1; t < T; ++t) {
    if (path[t - 1] != g || path[t] != g) ++c;
  }
  return c;
}

int get_sum_of_loss(const Solution &solution, std::vector<int> &agents_subset)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto T = solution.size();
  for (const auto i : agents_subset) {
    auto g = solution.back()[i];
    for (size_t t = 1; t < T; ++t) {
      if (solution[t - 1][i] != g || solution[t][i] != g) ++c;
    }
  }
  return c;
}

int get_sum_of_loss(const Solution &solution)
{
  if (solution.empty()) return 0;
  int c = 0;
  const auto N = solution.front().size();
  const auto T = solution.size();
  for (size_t i = 0; i < N; ++i) {
    auto g = solution.back()[i];
    for (size_t t = 1; t < T; ++t) {
      if (solution[t - 1][i] != g || solution[t][i] != g) ++c;
    }
  }
  return c;
}

int get_sum_of_loss_paths(const std::vector<Path> &solution)
{
  auto c = 0;
  for (auto &&path : solution) c += get_path_loss(path);
  return c;
}

int get_makespan_lower_bound(const Instance &ins,
                             DistTableMultiGoal &dist_table)
{
  int c = 0;
  for (size_t i = 0; i < ins.N; ++i) {
    int c_i = 0;
    for (size_t j = 1; j < ins.goal_sequences[i].size(); ++j) {
      c_i += dist_table.get(i, j, ins.goal_sequences[i][j - 1]);
    }
    c = std::max(c, c_i);
  }
  return c;
}

int get_sum_of_costs_lower_bound(const Instance &ins,
                                 DistTableMultiGoal &dist_table)
{
  int c = 0;
  for (size_t i = 0; i < ins.N; ++i) {
    for (size_t j = 1; j < ins.goal_sequences[i].size(); ++j) {
      c += dist_table.get(i, j, ins.goal_sequences[i][j - 1]);
    }
  }
  return c;
}

} // namespace lacam
