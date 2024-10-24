#include "../include/scatter.hpp"

#include "../include/metrics.hpp"

namespace lacam
{  

  Scatter::Scatter(const Instance *_ins, DistTableMultiGoal *_D,
                   const Deadline *_deadline, const int seed, int _verbose,
                   int _cost_margin)
      : ins(_ins),
        deadline(_deadline),
        MT(std::mt19937(seed)),
        verbose(_verbose),
        N(ins->N),
        V_size(ins->G->size()),
        T(get_makespan_lower_bound(*ins, *_D) + _cost_margin),
        D(_D),
        cost_margin(_cost_margin),
        sum_of_path_length(0),
        paths(N),
        scatter_data(N),
        CT(ins)
  {
  }

  void Scatter::construct()
  {
    info(0, verbose, deadline, "scatter", "\tinvoked");

    // define path finding utilities
    struct ScatterNode {
      Vertex *vertex;
      int goal_index;
      int cost_to_come;
      int cost_to_go;
      int collision;
      ScatterNode *parent;

      ScatterNode(Vertex *_vertex, int _goal_index, int _cost_to_come,
                  int _cost_to_go, int _collision, ScatterNode *_parent)
          : vertex(_vertex),
            goal_index(_goal_index),
            cost_to_come(_cost_to_come),
            cost_to_go(_cost_to_go),
            collision(_collision),
            parent(_parent)
      {
      }
    };
    auto cmp = [&](ScatterNode* a, ScatterNode* b) {
      // collision
      if (a->collision != b->collision) return a->collision > b->collision;
      auto f_a = a->cost_to_come + a->cost_to_go;
      auto f_b = b->cost_to_come + b->cost_to_go;
      if (f_a != f_b) return f_a > f_b;
      return a->vertex->id < b->vertex->id;
    };

    struct SingleAgentConfig {
      int v_id;
      int goal_index;

      SingleAgentConfig(int _v_id, int _goal_index)
          : v_id(_v_id), goal_index(_goal_index)
      {
      }

      bool operator==(const SingleAgentConfig &other) const
      {
        return v_id == other.v_id && goal_index == other.goal_index;
      }
    };
    struct SingleAgentConfigHasher {
      std::size_t operator()(const SingleAgentConfig &config) const
      {
        return hash_two_ints(config.v_id, config.goal_index);
      }
    };

    // metrics
    auto collision_cnt_last = 0;
    auto paths_prev = std::vector<Path>();

    // main loop
    auto loop = 0;
    while (loop < 2 || CT.collision_cnt < collision_cnt_last) {
      ++loop;
      collision_cnt_last = CT.collision_cnt;

      // randomize planning order
      auto order = std::vector<int>(N, 0);
      std::iota(order.begin(), order.end(), 0);
      std::shuffle(order.begin(), order.end(), MT);

      // single-agent path finding for agent-i
      for (int _i = 0; _i < N; ++_i) {
        if (is_expired(deadline)) break;

        const auto i = order[_i];

        // upper bound of cost for entire goal sequence
        auto cost_ub = D->get(i, 0, ins->starts[i]) + cost_margin;
        for (size_t goal_idx = 1; goal_idx < ins->goal_sequences[i].size();
             ++goal_idx) {
          const auto prev_goal = ins->goal_sequences[i][goal_idx - 1];
          cost_ub += D->get(i, goal_idx, prev_goal) + cost_margin;
        }

        if (!paths[i].empty()) sum_of_path_length -= (paths[i].size() - 1);

        // clear cache
        CT.clearPath(i, paths[i]);

        // setup A*
        auto OPEN = std::priority_queue<ScatterNode*, std::vector<ScatterNode*>,
                                        decltype(cmp)>(cmp);
        const auto s_i = ins->starts[i];
        OPEN.push(new ScatterNode(s_i, 0, 0, D->get(i, 0, s_i), 0, nullptr));
        auto CLOSED = std::unordered_map<SingleAgentConfig, ScatterNode*, SingleAgentConfigHasher>();

        // Multi-Label A*
        // arbitrary number of labels (from RHCR paper)
        while (!OPEN.empty() && !is_expired(deadline)) {
          // pop
          auto node = OPEN.top();
          OPEN.pop();

          // check CLOSED list
          const auto v = node->vertex;
          const auto gi = node->goal_index;
          const auto g_v = node->cost_to_come;
          const auto c_v = node->collision;
          const auto sa_c = SingleAgentConfig(v->id, gi);
          if (CLOSED[sa_c] != nullptr) continue;
          CLOSED[sa_c] = node;

          // update goal index
          if (v == ins->goal_sequences[i][gi]) {
            node->goal_index++;
            
            // check goal condition
            if (node->goal_index == ins->goal_sequences[i].size()) {             
              while (node != nullptr) {
                paths[i].push_back(node->vertex);
                node = node->parent;
              }
              std::reverse(paths[i].begin(), paths[i].end());
              break;
            }
          }

          // expand
          for (auto u : v->neighbor) {
            auto d_u = D->get(i, node->goal_index, u);
            auto sa_c = SingleAgentConfig(u->id, node->goal_index);
            if (CLOSED[sa_c] == nullptr && d_u + g_v + 1 <= cost_ub) {
              // insert new node
              OPEN.push(new ScatterNode(u, node->goal_index, g_v + 1, d_u,
                                        CT.getCollisionCost(v, u, g_v) + c_v, node));
            }
          }
        }

        // register to CT & update collision count
        CT.enrollPath(i, paths[i]);
        sum_of_path_length += paths[i].size() - 1;

        // memory management
        for (auto kv : CLOSED) {
          delete kv.second;
        }
      }

      paths_prev = paths;
      info(1, verbose, deadline, "scatter", "\titer:", loop,
           "\tcollision_cnt:", CT.collision_cnt);

      if (CT.collision_cnt == 0) break;
      if (is_expired(deadline)) break;
    }

    paths = paths_prev;

    // set scatter data
    for (auto i = 0; i < N; ++i) {
      if (paths[i].empty()) continue;
      for (size_t t = 0; t < paths[i].size() - 1; ++t) {
        scatter_data[i][paths[i][t]->id] = paths[i][t + 1];
      }
    }

    info(0, verbose, deadline, "scatter", "\tcompleted");
  }

}  // namespace lacam
