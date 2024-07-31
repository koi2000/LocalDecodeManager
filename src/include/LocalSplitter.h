#ifndef LOCAL_SPLITTER_H
#define LOCAL_SPLITTER_H

#include "EncodeBoundarySchema.h"
#include "core.h"
#include <Graph.h>
#include <cstring>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#define RANDOM_SEED 1035

/**
 * 内存中需要构建一个图结构，点是subMesh，边指的是两个subMesh之间有邻接关系
 */
class LocalSplitter {
  public:
    LocalSplitter() = default;

    LocalSplitter(std::string filename);

    void buildGraph();

    LocalSplitter(MCGAL::Mesh* mesh, bool skipMarkBoundary = false);

    void loadMesh(MCGAL::Mesh* mesh, bool skipMarkBoundary = false);

    void split(int groupNumber = -1);

    void dumpSubMesh(std::string path, int groupId);

    std::vector<MCGAL::Halfedge*>& exportSeeds();

    Graph exportGraph();

    std::vector<MCGAL::Mesh>& exportSubMeshes();

    std::unordered_map<int, int>& exportDup2Origin();

    std::unordered_map<int, std::unordered_map<int, int>>& exportOrigin2Dup();

  private:
    void markBoundry();

  private:
    MCGAL::Mesh* mesh = nullptr;
    std::vector<int> unRemovedPoint;
    std::vector<MCGAL::Mesh> subMeshes;
    std::vector<MCGAL::Halfedge*> seeds;
    std::vector<std::set<int>> triJunction;
    std::unordered_map<int, int> dup2origin;
    /**
     * key为groupId
     * value 为origin到新点的集合，其中有pair first为groupId，second为dup的id
     */
    std::unordered_map<int, std::unordered_map<int, int>> origin2dup;
    Graph g;
};

#endif