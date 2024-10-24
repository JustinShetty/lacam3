#include <cassert>
#include <lacam.hpp>

using namespace lacam;

int main()
{
  const int VERBOSITY = 0;

  // two agents, one goal each
  {
    const auto map_filename = "../assets/empty-8-8.map";
    // upper left corner
    //  0  1  2
    //  8  9 10
    // 16 17 18

    const std::vector<int> starts = {8, 17};
    std::vector<std::vector<int>> goal_sequences = {{18}, {1}};
    const auto ins = Instance(map_filename, starts, goal_sequences);
    assert(ins.is_valid(VERBOSITY));

    DistTableMultiGoal D = DistTableMultiGoal(&ins);
    const auto sc_margin = 0;
    Scatter sc = Scatter(&ins, &D, nullptr, 0, VERBOSITY, sc_margin);
    sc.construct();

    std::vector<std::vector<int>> expected = {{8, 16, 17, 18}, {17, 9, 1}};
    for (int i = 0; i < ins.N; ++i) {
      for (int j = 0; j < sc.paths[i].size(); ++j) {
        assert(sc.paths[i][j]->id == expected[i][j]);
      }
    }
  }

  // single agent, multiple goals
  {
    const auto map_filename = "../tests/assets/2x2.map";
    // 0 1
    // 2 3

    const std::vector<int> starts = {0};
    std::vector<std::vector<int>> goal_sequences = {{3, 0}};
    const auto ins = Instance(map_filename, starts, goal_sequences);
    assert(ins.is_valid(VERBOSITY));

    DistTableMultiGoal D = DistTableMultiGoal(&ins);
    const auto sc_margin = 0;
    Scatter sc = Scatter(&ins, &D, nullptr, 0, VERBOSITY, sc_margin);
    sc.construct();

    assert(sc.paths[0].size() == 5);
  }

  return 0;
}
