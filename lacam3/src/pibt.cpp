#include "../include/pibt.hpp"

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
                          const std::vector<int> &order, const int search_iter)
{
  bool success = true;
  // setup cache & constraints check
  for (auto i = 0; i < N; ++i) {
    // set occupied now
    occupied_now[Q_from[i]->id] = i;

    // set occupied next
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
      if (Q_to[i] == nullptr && !funcPIBT(i, NO_AGENT, Q_from, Q_to, search_iter)) {
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
                    Config &Q_to, const int search_iter)
{
  const auto K = Q_from[i]->neighbor.size();

  // exploit scatter data
  // Vertex *prioritized_vertex = nullptr;
  // if (scatter != nullptr) {
  //   auto itr_s = scatter->scatter_data[i].find(Q_from[i]->id);
  //   if (itr_s != scatter->scatter_data[i].end()) {
  //     prioritized_vertex = itr_s->second;
  //   }
  // }

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
  std::sort(C_next[i].begin(), C_next[i].begin() + num_candidates,
            [&](Vertex *const v, Vertex *const u) {
              // if (v == prioritized_vertex) return true;
              // if (u == prioritized_vertex) return false;
              return D->get(i, Q_from.goal_indices[i], v) +
                         tie_breakers[v->id] <
                     D->get(i, Q_from.goal_indices[i], u) + tie_breakers[u->id];
            });

  // emulate swap
  // auto swap_agent = NO_AGENT;
  // if (flg_swap) {
  //   swap_agent = is_swap_required_and_possible(i, Q_from, Q_to);
  //   if (swap_agent != NO_AGENT) {
  //     // reverse vertex scoring
  //     std::reverse(C_next[i].begin(), C_next[i].begin() + num_candidates);
  //   }
  // }

  // auto swap_operation = [&]() {
  //   throw std::runtime_error("swap operation not implemented");
  //   if (swap_agent != NO_AGENT &&                 // swap_agent exists
  //       Q_to[swap_agent] == nullptr &&            // not decided
  //       occupied_next[Q_from[i]->id] == NO_AGENT  // free
  //   ) {
  //     // pull swap_agent
  //     occupied_next[Q_from[i]->id] = swap_agent;
  //     Q_to[swap_agent] = Q_from[i];
  //   }
  // };

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
          info(1, 1, "[", search_iter, "] agent-", i, " trying to preemptively reserve vertex-", u->id, ", but it is already reserved by agent-", occupied_next[Q_from[i]->id]);
        }
        occupied_next[Q_from[i]->id] = i;
        Q_to[i] = Q_from[i];

        if (funcPIBT(j, i, Q_from, Q_to, search_iter)) {
          if (i == 255) info(1,1, "[", search_iter, "] agent-", i, " moves to vertex-", Q_to[i]->id);
          return true;
        }

        // revert if priority inheritance failed
        occupied_next[Q_from[i]->id] = NO_AGENT;
        Q_to[i] = nullptr;
      }
      continue;
    }

    // success
    occupied_next[u->id] = i;
    Q_to[i] = u;
    if (i == 255) info(1,1, "[", search_iter, "] agent-", i, " moves to vertex-", Q_to[i]->id);
    // if (flg_swap && k == 0) swap_operation();
    return true;
  }

  // failed to secure node, remain at current location
  occupied_next[Q_from[i]->id] = i;
  Q_to[i] = Q_from[i];
  if (i == 255) info(1,1, "[", search_iter, "] agent-", i, " moves to vertex-", Q_to[i]->id);
  return false;
}

int PIBT::is_swap_required_and_possible(const int i, const Config &Q_from,
                                        Config &Q_to)
{
  // agent-j occupying the desired vertex for agent-i
  const auto j = occupied_now[C_next[i][0]->id];
  if (j != NO_AGENT && j != i &&  // j exists
      Q_to[j] == nullptr &&       // j does not decide next location
      is_swap_required(i, j, Q_from, Q_from[i], Q_from[j]) &&  // swap required
      is_swap_possible(Q_from[j], Q_from[i])                   // swap possible
  ) {
    return j;
  }

  // for clear operation, c.f., push & swap
  if (C_next[i][0] != Q_from[i]) {
    for (auto u : Q_from[i]->neighbor) {
      const auto k = occupied_now[u->id];
      if (k != NO_AGENT &&              // k exists
          C_next[i][0] != Q_from[k] &&  // this is for clear operation
          is_swap_required(k, i, Q_from, Q_from[i],
                           C_next[i][0]) &&  // emulating from one step ahead
          is_swap_possible(C_next[i][0], Q_from[i])) {
        return k;
      }
    }
  }
  return NO_AGENT;
}

bool PIBT::is_swap_required(const int pusher, const int puller, const Config &Q,
                            Vertex *v_pusher_origin, Vertex *v_puller_origin)
{
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex *tmp = nullptr;
  while (D->get(pusher, Q.goal_indices[pusher], v_puller) <
         D->get(pusher, Q.goal_indices[pusher], v_pusher)) {
    auto n = v_puller->neighbor.size();
    // remove agents who need not to move
    for (auto u : v_puller->neighbor) {
      const auto i = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && i != NO_AGENT && ins->goals[i] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return false;  // able to swap at v_l
    if (n <= 0) break;
    v_pusher = v_puller;
    v_puller = tmp;
  }

  return (D->get(puller, Q.goal_indices[puller], v_pusher) <
          D->get(puller, Q.goal_indices[puller], v_puller)) &&
         (D->get(pusher, Q.goal_indices[pusher], v_pusher) == 0 ||
          D->get(pusher, Q.goal_indices[pusher], v_puller) <
              D->get(pusher, Q.goal_indices[pusher], v_pusher));
}

bool PIBT::is_swap_possible(Vertex *v_pusher_origin, Vertex *v_puller_origin)
{
  // simulate pull
  auto v_pusher = v_pusher_origin;
  auto v_puller = v_puller_origin;
  Vertex *tmp = nullptr;
  while (v_puller != v_pusher_origin) {  // avoid loop
    auto n = v_puller->neighbor.size();
    for (auto u : v_puller->neighbor) {
      const auto i = occupied_now[u->id];
      if (u == v_pusher ||
          (u->neighbor.size() == 1 && i != NO_AGENT && ins->goals[i] == u)) {
        --n;
      } else {
        tmp = u;
      }
    }
    if (n >= 2) return true;  // able to swap at v_next
    if (n <= 0) return false;
    v_pusher = v_puller;
    v_puller = tmp;
  }
  return false;
}
