#include "../include/graph.hpp"

namespace lacam
{

  Vertex::Vertex(int _id, int _index, int _x, int _y)
      : id(_id), index(_index), x(_x), y(_y), neighbor()
  {
  }

  std::string Vertex::to_str() const
  {
      return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
  }

  Graph::Graph() : V(Vertices()), width(0), height(0) {}

  Graph::~Graph()
  {
    for (auto &v : V)
      if (v != nullptr) delete v;
    V.clear();
  }

  // to load graph
  static const std::regex r_height = std::regex(R"(height\s(\d+))");
  static const std::regex r_width = std::regex(R"(width\s(\d+))");
  static const std::regex r_map = std::regex(R"(map)");

  Graph::Graph(const std::string &filename) : V(Vertices()), width(0), height(0)
  {
    std::ifstream file(filename);
    if (!file) {
      std::cout << "file " << filename << " is not found." << std::endl;
      return;
    }
    std::string line;
    std::smatch results;

    // read fundamental graph parameters
    while (getline(file, line)) {
      // for CRLF coding
      if (*(line.end() - 1) == 0x0d) line.pop_back();

      if (std::regex_match(line, results, r_height)) {
        height = std::stoi(results[1].str());
      }
      if (std::regex_match(line, results, r_width)) {
        width = std::stoi(results[1].str());
      }
      if (std::regex_match(line, results, r_map)) break;
    }

    U = Vertices(width * height, nullptr);

    // create vertices
    int y = 0;
    while (getline(file, line)) {
      // for CRLF coding
      if (*(line.end() - 1) == 0x0d) line.pop_back();
      for (int x = 0; x < width; ++x) {
        char s = line[x];
        if (s == 'T' or s == '@') continue;  // object
        auto index = width * y + x;
        auto v = new Vertex(V.size(), index, x, y);
        V.push_back(v);
        U[index] = v;
      }
      ++y;
    }
    file.close();

    // create edges
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        auto v = U[width * y + x];
        if (v == nullptr) continue;
        // left
        if (x > 0) {
          auto u = U[width * y + (x - 1)];
          if (u != nullptr) v->neighbor.push_back(u);
        }
        // right
        if (x < width - 1) {
          auto u = U[width * y + (x + 1)];
          if (u != nullptr) v->neighbor.push_back(u);
        }
        // up
        if (y < height - 1) {
          auto u = U[width * (y + 1) + x];
          if (u != nullptr) v->neighbor.push_back(u);
        }
        // down
        if (y > 0) {
          auto u = U[width * (y - 1) + x];
          if (u != nullptr) v->neighbor.push_back(u);
        }
      }
    }
  }

  int Graph::size() const { return V.size(); }

  bool is_same_config(const Config &C1, const Config &C2)
  {
    for (size_t i = 0; i < C1.size(); ++i) {
      if (C1[i]->id != C2[i]->id) return false;
    }
    return true;
  }

  bool enough_goals_reached(const Config &C, int threshold)
  {
    int count = 0;
    for (size_t i = 0; i < C.size(); ++i) {
      count += C.goal_indices[i];
      if (count >= threshold) return true;
    }
    return false;
  }

  uint ConfigHasher::operator()(const Config &C) const
  {
    uint location_hash = C.size();
    for (auto &v : C) {
      location_hash ^=
          v->id + 0x9e3779b9 + (location_hash << 6) + (location_hash >> 2);
    }
    uint indices_hash = C.goal_indices.size();
    for (auto &idx : C.goal_indices) {
      indices_hash ^=
          idx + 0x9e3779b9 + (indices_hash << 6) + (indices_hash >> 2);
    }
    return hash_two_ints(location_hash, indices_hash);
  }

  std::ostream &operator<<(std::ostream &os, const Vertex *v)
  {
    if (v == nullptr) {
      os << "nullptr";
    } else {
      os << v->index;
    }
    return os;
  }

  std::ostream &operator<<(std::ostream &os, const Config &Q)
  {
    os << "{ ";
    for (auto v : Q) os << v << " ";
    os << "} ";
    os << "{ ";
    for (auto i : Q.goal_indices) os << i << " ";
    os << "}";
    return os;
  }

  std::ostream &operator<<(std::ostream &os, const Paths &paths)
  {
    for (auto i = 0; i < paths.size(); ++i) {
      os << i << ":";
      for (auto &v : paths[i]) {
        os << std::setw(4) << v << "->";
      }
      std::cout << std::endl;
    }
    return os;
  }

  bool has_following_conflict(const Config &c_from, const Config &c_to)
  {
    std::unordered_map<Vertex *, int> occupied_from;
    for (size_t i = 0; i < c_from.size(); ++i) {
      occupied_from[c_from[i]] = i;
    }
    for (size_t i = 0; i < c_from.size(); ++i) {
      const auto was_there = occupied_from.find(c_to[i]);
      if (was_there != occupied_from.end() && was_there->second != i)
        return true;
    }
    return false;
  }

}  // namespace lacam
