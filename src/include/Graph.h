#ifndef GRAPH_H
#define GRAPH_H
#include <unordered_map>
#include <unordered_set>
class Graph {
  public:
    Graph() = default;

    void addHEdge(int u, int v) {
        g[u].insert(v);
    }

    void addEdge(int u, int v) {
        g[u].insert(v);
        g[v].insert(u);
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
    std::unordered_map<int, std::unordered_set<int>> g;
};

#endif