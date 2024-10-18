#include <cassert>
#include <lacam.hpp>

int main()
{
  Planner::FLG_SWAP = false;
  Planner::FLG_STAR = false;
  Planner::PIBT_NUM = 1;

  const int VERBOSITY = 0;

  // basic
  {
    const auto scen_filename = "../assets/random-32-32-10-random-1.scen";
    const auto map_filename = "../assets/random-32-32-10.map";
    const auto ins = Instance(scen_filename, map_filename, 3);
    assert(ins.is_valid(VERBOSITY));
    const auto threshold = ins.get_total_goals();

    auto solution = solve(ins, threshold, VERBOSITY, nullptr, 0);
    assert(solution.size() > 0);
    assert(is_feasible_solution(ins, solution, threshold, VERBOSITY));
  }

  // no solution
  {
    const auto scen_filename = "../tests/assets/2x1.scen";
    const auto map_filename = "../tests/assets/2x1.map";
    const auto ins = Instance(scen_filename, map_filename, 2);
    assert(ins.is_valid(VERBOSITY));
    const auto threshold = ins.get_total_goals();

    auto solution = solve(ins, threshold, VERBOSITY, nullptr, 0);
    assert(solution.empty());
  }

  // multiple goals
  {
    const auto map_filename = "../tests/assets/2x2.map";

    const std::vector<int> starts = {0};
    std::vector<std::vector<int>> goal_sequences = {{3, 0}};
    const auto ins = Instance(map_filename, starts, goal_sequences);
    assert(ins.is_valid(VERBOSITY));
    const auto threshold = ins.get_total_goals();

    auto solution = solve(ins, threshold, VERBOSITY, nullptr, 0);
    assert(solution.size() > 0);
    assert(is_feasible_solution(ins, solution, threshold, VERBOSITY));
  }

  // multiple goals, multiple agents
  {
    const auto map_filename = "../tests/assets/2x2.map";

    const std::vector<int> starts = {0, 3};
    std::vector<std::vector<int>> goal_sequences = {{3, 0}, {0, 3}};
    const auto ins = Instance(map_filename, starts, goal_sequences);
    assert(ins.is_valid(VERBOSITY));
    const auto threshold = ins.get_total_goals();

    auto solution = solve(ins, threshold, VERBOSITY, nullptr, 0);
    assert(solution.size() > 0);
    assert(is_feasible_solution(ins, solution, threshold, VERBOSITY));
  }

  // 32x32, multiple goals, 3 agents
  {
    const auto map_filename = "../assets/random-32-32-10.map";

    const std::vector<int> starts = {174, 662, 0};
    std::vector<std::vector<int>> goal_sequences = {{0, 1022}, {992}, {1023}};
    const auto ins = Instance(map_filename, starts, goal_sequences);
    assert(ins.is_valid(VERBOSITY));

    const auto threshold = ins.get_total_goals();
    auto solution = solve(ins, threshold, VERBOSITY, nullptr, 0);
    assert(solution.size() > 0);
    assert(is_feasible_solution(ins, solution, threshold, VERBOSITY));
  }

  return 0;
}
