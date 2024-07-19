#ifndef LOCAL_SPLITTER_H
#define LOCAL_SPLITTER_H

#include "EncodeBoundarySchema.h"
#include "biops.h"
#include "core.h"
#include "syncbitops.h"
#include <Graph.h>
#include <cstring>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>

#define RANDOM_SEED 1035

/**
 * 内存中需要构建一个图结构，点是subMesh，边指的是两个subMesh之间有邻接关系
 */
class LocalSplitter {
  public:
    LocalSplitter() = default;

    LocalSplitter(std::string filename);

    LocalSplitter(MCGAL::Mesh* mesh);

    void loadMesh(MCGAL::Mesh* mesh);

    void split();

    void dumpSubMesh(std::string path, int groupId);

    std::vector<MCGAL::Halfedge*>& exportSeeds();

    Graph exportGraph();

    std::vector<MCGAL::Mesh>& exportSubMeshes();

  private:
    void markBoundry();

  private:
    MCGAL::Mesh* mesh = nullptr;
    std::vector<int> unRemovedPoint;
    std::vector<MCGAL::Mesh> subMeshes;
    std::vector<MCGAL::Halfedge*> seeds;
    Graph g;
};

#endif