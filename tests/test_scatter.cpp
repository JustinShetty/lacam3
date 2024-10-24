#include <cassert>
#include <lacam.hpp>

using namespace lacam;

int main()
{
  const int VERBOSITY = 0;

  // single agent, multiple goals
  {
    const auto map_filename = "../tests/assets/2x2.map";

    const std::vector<int> starts = {0};
    std::vector<std::vector<int>> goal_sequences = {{3, 0}};
    const auto ins = Instance(map_filename, starts, goal_sequences);
    assert(ins.is_valid(VERBOSITY));

    DistTableMultiGoal D = DistTableMultiGoal(&ins);
    const auto sc_margin = 0;
    Scatter sc = Scatter(&ins, &D, nullptr, 0, VERBOSITY, sc_margin);
    sc.construct();
    
    for (int i = 0; i < ins.N; ++i) {
      std::cout << "sc.paths[" << i << "]: ";
      for (const auto& v : sc.paths[i]) {
        std::cout << v->id << " ";
      }
      std::cout << std::endl;
    }
  }

  return 0;
}
