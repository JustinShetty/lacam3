/*
 * graph definition
 */
#pragma once
#include <iterator>

#include "utils.hpp"

struct Vertex {
  const int id;     // index for V in Graph
  const int index;  // index for U (width * y + x) in Graph
  const int x;
  const int y;
  std::vector<Vertex *> neighbor;

  Vertex(int _id, int _index, int _x, int _y);
};
using Vertices = std::vector<Vertex *>;
class Config : public std::vector<Vertex *>
{
public:
  Config() : std::vector<Vertex *>(), goal_indices() {}
  Config(const int N, Vertex *v)
      : std::vector<Vertex *>(N, v), goal_indices(N, 0)
  {
  }
  Config(const std::initializer_list<Vertex *> vertices)
      : std::vector<Vertex *>(vertices), goal_indices(vertices.size(), 0)
  {
  }
  Config(const std::initializer_list<Vertex *> vertices,
         const std::initializer_list<int> goal_indices)
      : std::vector<Vertex *>(vertices), goal_indices(goal_indices)
  {
  }

  bool operator==(const Config &C) const
  {
    if (this->size() != C.size()) return false;
    for (size_t i = 0; i < this->size(); ++i) {
      if (this->at(i) != C.at(i)) return false;
    }
    for (size_t i = 0; i < goal_indices.size(); i++) {
      if (goal_indices[i] != C.goal_indices[i]) return false;
    }
    return true;
  }

  bool operator!=(const Config &C) const { return !(*this == C); }

  void push_back(Vertex *v, int goal_index)
  {
    std::vector<Vertex *>::push_back(v);
    goal_indices.push_back(goal_index);
  }

  std::vector<int> goal_indices;
};
std::ostream &operator<<(std::ostream &os, const Config &c);
using Path = std::vector<Vertex *>;  // path
using Paths = std::vector<Path>;

struct Graph {
  Vertices V;  // without nullptr
  Vertices U;  // with nullptr, i.e., |U| = width * height
  int width;   // grid width
  int height;  // grid height
  Graph();
  Graph(const std::string &filename);  // taking map filename
  ~Graph();

  int size() const;  // the number of vertices, |V|
};

inline int manhattanDist(Vertex *a, Vertex *b)
{
  return std::abs(a->x - b->x) + std::abs(a->y - b->y);
}

bool is_same_config(
    const Config &C1,
    const Config &C2);  // check equivalence of two configurations

bool enough_goals_reached(const Config &C, const int threshold);

// hash function of configuration
// c.f.
// https://stackoverflow.com/questions/10405030/c-unordered-map-fail-when-used-with-a-vector-as-key
struct ConfigHasher {
  uint operator()(const Config &C) const;
};

std::ostream &operator<<(std::ostream &os, const Vertex *v);
std::ostream &operator<<(std::ostream &os, const Config &Q);
std::ostream &operator<<(std::ostream &os, const Paths &paths);

bool has_following_conflict(const Config &c_from, const Config &c_to);
