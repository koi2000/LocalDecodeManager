#ifndef LOCAL_SPLITTER_H
#define LOCAL_SPLITTER_H

#include "EncodeBoundarySchema.h"
#include "biops.h"
#include "core.h"
#include "syncbitops.h"
#include <cstring>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>

#define RANDOM_SEED 1035

class LocalSplitter {
  public:
    LocalSplitter(std::string filename);

    void split();

    void dumpSubMesh(std::string path, int groupId);

  private:
    void markBoundry();

  private:
    MCGAL::Mesh mesh;
    std::vector<MCGAL::Mesh> subMeshes;
    std::vector<MCGAL::Halfedge*> seeds;
};

#endif