#include "../include/dist_table.hpp"

namespace lacam {

DistTableMultiGoal::DistTableMultiGoal(const Instance *ins)
    : K(ins->G->V.size())
{
  setup(ins);
}

void DistTableMultiGoal::setup(const Instance *ins)
{
  // initialize all values to K
  for (size_t i = 0; i < ins->N; i++) {
    table.push_back(std::vector<std::vector<int>>(ins->goal_sequences[i].size(),
                                                  std::vector<int>(K, K)));
  }

  auto bfs = [&](const int i, const int j) {
    const auto g_ij = ins->goal_sequences[i][j];
    auto Q = std::queue<Vertex *>({g_ij});
    table[i][j][g_ij->id] = 0;
    while (!Q.empty()) {
      const auto n = Q.front();
      Q.pop();
      const int d_n = table[i][j][n->id];
      for (auto &m : n->neighbor) {
        const int d_m = table[i][j][m->id];
        if (d_n + 1 >= d_m) continue;
        table[i][j][m->id] = d_n + 1;
        Q.push(m);
      }
    }
  };

  auto pool = std::vector<std::future<void>>();
  for (size_t i = 0; i < ins->N; ++i) {
    for (size_t j = 0; j < ins->goal_sequences[i].size(); ++j) {
      pool.emplace_back(std::async(std::launch::async, bfs, i, j));
    }
  }
}

int DistTableMultiGoal::get(const int i, const int goal_index,
                            const int v_id) const
{
  // goal_index can be past the end to signify we've already reached the last
  // goal, but when we want to use the index we need to cap it at the last goal
  auto idx = std::min(goal_index, (int)(table[i].size() - 1));
  return table[i][idx][v_id];
}

}  // namespace lacam
