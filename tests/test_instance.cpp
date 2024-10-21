#include <cassert>
#include <lacam.hpp>

using namespace lacam;

int main()
{
  {
    const auto scen_filename = "../assets/random-32-32-10-random-1.scen";
    const auto map_filename = "../assets/random-32-32-10.map";
    const auto ins = Instance(scen_filename, map_filename, 3);

    assert(size(ins.starts) == 3);
    assert(size(ins.goals) == 3);
    assert(ins.starts[0]->index == 203);
    assert(ins.goals[0]->index == 583);
  }

  {
    const auto map_filename = "../assets/empty-8-8.map";
    const auto ins = Instance(map_filename, {0, 1}, {{2, 3, 4}, {5, 6, 7}});

    assert(size(ins.starts) == 2);
    assert(size(ins.goals) == 2);
    assert(ins.starts[0]->index == 0);
    assert(ins.goals[0]->index == 4);

    assert(ins.goal_sequences.size() == 2);
    assert(ins.goal_sequences[0].front()->index == 2);
    assert(ins.goal_sequences[0].back()->index == 4);
    assert(ins.goal_sequences[1].front()->index == 5);
    assert(ins.goal_sequences[1].back()->index == 7);

    Config goals({ins.G->U[4], ins.G->U[7]});
    goals.goal_indices = {2, 2};
    assert(ins.goals == goals);
  }

  return 0;
}
