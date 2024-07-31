#ifndef GRAPH_H
#define GRAPH_H
#include <unordered_map>
#include <unordered_set>

class Node {
  public:
    int st;
    int ed;
    int neighbour;
    int stop;
    Node(int st_, int ed_, int neighbour_, int stop_) : st(st_), ed(ed_), neighbour(neighbour_), stop(stop_) {}

    bool operator==(const Node& other) const {
        return st == other.st && ed == other.ed && neighbour == other.neighbour;
    }
};

class Hasher {
  public:
    size_t operator()(const Node& n) const {
        return std::hash<int>()(n.st) + std::hash<int>()(n.ed) + std::hash<int>()(n.neighbour);
    }
};

class Graph {
  public:
    Graph() = default;

    void addHEdge(int u, Node& v) {
        g[u].insert(v);
    }

    void addEdge(Node& u, Node& v) {
        g[u.neighbour].insert(v);
        g[v.neighbour].insert(u);
    }

    void addEdge(Node&& u, Node&& v) {
        g[u.neighbour].insert(v);
        g[v.neighbour].insert(u);
    }

    std::unordered_set<Node, Hasher>& getNode(int id) {
        return g[id];
    }

    // 移动构造函数
    Graph(Graph&& other) noexcept : g(std::move(other.g)) {}

    // 移动赋值运算符
    Graph& operator=(Graph&& other) noexcept {
        if (this != &other) {
            g = std::move(other.g);
        }
        return *this;
    }

  private:
    std::unordered_map<int, std::unordered_set<Node, Hasher>> g;
};

#endif