#include "../include/instance.hpp"

namespace lacam
{

  Instance::~Instance()
  {
    if (delete_graph_after_used) delete G;
  }

  Instance::Instance(Graph *_G, const Config &_starts,
                     const std::vector<std::vector<Vertex *>> &_goal_sequences,
                     uint _N)
      : G(_G),
        starts(_starts),
        goal_sequences(_goal_sequences),
        N(_N)
  {
  }

  Instance::Instance(const std::string &map_filename,
                     const std::vector<int> &start_indexes,
                     const std::vector<int> &goal_indexes)
      : G(new Graph(map_filename)),
        starts(Config()),
        N(start_indexes.size()),
        delete_graph_after_used(true)
  {
    for (auto k : start_indexes) starts.push_back(G->U[k], 0);
    for (auto k : goal_indexes) {
      const auto vp = G->U[k];
      goal_sequences.push_back(std::vector<Vertex *>{vp});
    }
  }

  Instance::Instance(const std::string &map_filename,
                     const std::vector<int> &start_indexes,
                     const std::vector<std::vector<int>> &goal_index_sequences)
      : G(new Graph(map_filename)),
        starts(Config()),
        N(start_indexes.size())
  {
    if (start_indexes.size() != N) {
      throw std::invalid_argument("start_indexes and N size mismatch");
    }
    if (goal_index_sequences.size() != N) {
      throw std::invalid_argument("goal_index_sequences and N size mismatch");
    }
    for (auto k : start_indexes) starts.push_back(G->U[k], 0);
    for (auto goal_sequence : goal_index_sequences) {
      if (goal_sequence.empty()) {
        throw std::invalid_argument("encountered empty goal_sequence");
      }
      std::vector<Vertex *> as_vertices;
      for (auto k : goal_sequence) as_vertices.push_back(G->U[k]);
      goal_sequences.push_back(as_vertices);
    }
  }

  // for load instance
  static const std::regex r_instance =
      std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

  Instance::Instance(const std::string &scen_filename,
                     const std::string &map_filename, const int _N)
      : G(new Graph(map_filename)),
        starts(Config()),
        N(_N),
        delete_graph_after_used(true)
  {
    // load start-goal pairs
    std::ifstream file(scen_filename);
    if (!file) {
      info(0, 0, scen_filename, " is not found");
      return;
    }
    std::string line;
    std::smatch results;

    while (getline(file, line)) {
      // for CRLF coding
      if (*(line.end() - 1) == 0x0d) line.pop_back();

      if (std::regex_match(line, results, r_instance)) {
        auto x_s = std::stoi(results[1].str());
        auto y_s = std::stoi(results[2].str());
        auto x_g = std::stoi(results[3].str());
        auto y_g = std::stoi(results[4].str());
        if (x_s < 0 || G->width <= x_s || x_g < 0 || G->width <= x_g) continue;
        if (y_s < 0 || G->height <= y_s || y_g < 0 || G->height <= y_g)
          continue;
        auto s = G->U[G->width * y_s + x_s];
        auto g = G->U[G->width * y_g + x_g];
        if (s == nullptr || g == nullptr) continue;
        starts.push_back(s, 0);
        goal_sequences.push_back(std::vector<Vertex *>{g});
      }

      if (starts.size() == N) break;
    }
  }

  Instance::Instance(const std::string &map_filename, const int _N,
                     const int seed)
      : G(new Graph(map_filename)),
        starts(Config()),
        N(_N),
        delete_graph_after_used(true)
  {
    auto MT = std::mt19937(seed);
    // random assignment
    const auto K = G->size();

    // set starts
    auto s_indexes = std::vector<int>(K);
    std::iota(s_indexes.begin(), s_indexes.end(), 0);
    std::shuffle(s_indexes.begin(), s_indexes.end(), MT);
    int i = 0;
    while (true) {
      if (i >= K) return;
      starts.push_back(G->V[s_indexes[i]], 0);
      if (starts.size() == N) break;
      ++i;
    }

    // set goals
    auto g_indexes = std::vector<int>(K);
    std::iota(g_indexes.begin(), g_indexes.end(), 0);
    std::shuffle(g_indexes.begin(), g_indexes.end(), MT);
    int j = 0;
    while (true) {
      if (j >= K) return;
      const auto vp = G->V[g_indexes[j]];
      goal_sequences.push_back(std::vector<Vertex *>{vp});
      if (goal_sequences.size() == N) break;
      ++j;
    }
  }

  bool Instance::is_valid(const int verbose) const
  {
    if (N != starts.size() || N != goal_sequences.size()) {
      info(1, verbose, "invalid N, check instance");
      return false;
    }
    return true;
  }

  int Instance::get_total_goals() const
  {
    int total_goals = 0;
    for (const auto &goals : goal_sequences) {
      total_goals += goals.size();
    }
    return total_goals;
  }

}  // namespace lacam
