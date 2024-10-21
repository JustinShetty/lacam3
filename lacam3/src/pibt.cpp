#include "../include/pibt.hpp"

namespace lacam
{

  PIBT::PIBT(const Instance *_ins, DistTableMultiGoal *_D, int seed,
             bool _flg_swap, Scatter *_scatter)
      : ins(_ins),
        MT(std::mt19937(seed)),
        N(ins->N),
        V_size(ins->G->size()),
        D(_D),
        NO_AGENT(N),
        occupied_now(V_size, NO_AGENT),
        occupied_next(V_size, NO_AGENT),
        C_next(N, std::array<Vertex *, 5>()),
        tie_breakers(V_size, 0),
        flg_swap(_flg_swap),
        scatter(_scatter)
  {
  }

  PIBT::~PIBT() {}

  bool PIBT::set_new_config(const Config &Q_from, Config &Q_to,
                            const std::vector<int> &order)
  {
    bool success = true;

    // set occupied_now (must all be set before checking constraints to properly
    // check for following conflicts)
    for (auto i = 0; i < N; ++i) {
      occupied_now[Q_from[i]->id] = i;
    }

    // constraints check and set occupied_next
    for (auto i = 0; i < N; ++i) {
      if (Q_to[i] != nullptr) {
        // vertex conflict
        if (occupied_next[Q_to[i]->id] != NO_AGENT) {
          success = false;
          break;
        }
        // following conflict
        if (occupied_now[Q_to[i]->id] != NO_AGENT &&
            occupied_now[Q_to[i]->id] != i) {
          success = false;
          break;
        }
        occupied_next[Q_to[i]->id] = i;
      }
    }

    if (success) {
      for (auto i : order) {
        if (Q_to[i] == nullptr && !funcPIBT(i, NO_AGENT, Q_from, Q_to)) {
          success = false;
          break;
        }
      }
    }

    // cleanup
    for (auto i = 0; i < N; ++i) {
      occupied_now[Q_from[i]->id] = NO_AGENT;
      if (Q_to[i] != nullptr) occupied_next[Q_to[i]->id] = NO_AGENT;
    }

    return success;
  }

  bool PIBT::funcPIBT(const int i, const int i_caller, const Config &Q_from,
                      Config &Q_to)
  {
    const auto K = Q_from[i]->neighbor.size();

    // exploit scatter data
    Vertex *prioritized_vertex = nullptr;
    if (scatter != nullptr) {
      auto itr_s = scatter->scatter_data[i].find(Q_from[i]->id);
      if (itr_s != scatter->scatter_data[i].end()) {
        prioritized_vertex = itr_s->second;
      }
    }

    // set C_next
    for (size_t k = 0; k < K; ++k) {
      auto u = Q_from[i]->neighbor[k];
      C_next[i][k] = u;
      tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
    }
    size_t num_candidates = K;
    if (i_caller == NO_AGENT) {
      C_next[i][K] = Q_from[i];
      num_candidates++;
    }

    // sort C_next
    std::sort(
        C_next[i].begin(), C_next[i].begin() + num_candidates,
        [&](Vertex *const v, Vertex *const u) {
          if (v == prioritized_vertex) return true;
          if (u == prioritized_vertex) return false;
          return D->get(i, Q_from.goal_indices[i], v) + tie_breakers[v->id] <
                 D->get(i, Q_from.goal_indices[i], u) + tie_breakers[u->id];
        });

    // main loop
    for (size_t k = 0; k < num_candidates; ++k) {
      auto u = C_next[i][k];

      // avoid vertex conflicts
      if (occupied_next[u->id] != NO_AGENT) continue;

      // avoid following conflicts
      const auto j = occupied_now[u->id];
      if (j != NO_AGENT && j != i) {
        if (Q_to[j] == nullptr) {
          // preemptively reserve current location
          if (occupied_next[Q_from[i]->id] != NO_AGENT) {
            info(1, 1, "agent-", i, " trying to preemptively reserve vertex-",
                 u->id, ", but it is already reserved by agent-",
                 occupied_next[Q_from[i]->id]);
            return false;
          }
          occupied_next[Q_from[i]->id] = i;
          Q_to[i] = Q_from[i];

          if (funcPIBT(j, i, Q_from, Q_to)) return true;

          // revert if priority inheritance failed
          occupied_next[Q_from[i]->id] = NO_AGENT;
          Q_to[i] = nullptr;
        }
        continue;
      }

      // success
      occupied_next[u->id] = i;
      Q_to[i] = u;
      return true;
    }

    // failed to secure node, remain at current location
    occupied_next[Q_from[i]->id] = i;
    Q_to[i] = Q_from[i];
    return false;
  }

}  // namespace lacam
