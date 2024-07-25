#ifndef PARTIAL_DECODER_H
#define PARTIAL_DECODER_H

#include "EncodeBoundarySchema.h"
#include "Graph.h"
#include "LocalSplitter.h"
#include "biops.h"
#include "core.h"
#include "syncbitops.h"
#include <cstring>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

class PartialDecoder {
  public:
    PartialDecoder();

    PartialDecoder(std::string path);

    void decode();

    void decode(int groupId);

    void decode(int groupId, int lod);

  private:
    std::vector<int> decodeFacetSymbolOp(int groupId, std::vector<MCGAL::Halfedge*>& boundarys);

    std::vector<int> decodeHalfedgeSymbolOp(int groupId);

    void insertRemovedVertex(std::vector<int>& fids);

    void joinFacet(std::vector<int>& hids);

    void resetBfsState(int groupId);

    void resetState(int groupId);

    void readBoundary();

    void readMeta();

    void loadBuffer(std::string path);

    void readBaseMesh();

    void buildFromBuffer(std::deque<MCGAL::Point>* p_pointDeque, std::deque<uint32_t*>* p_faceDeque);

    void fillGroupId();

  private:
    LocalSplitter splitter;
    MCGAL::Mesh mesh;
    std::vector<MCGAL::Mesh> subMeshes;
    std::unordered_map<int, int> dup2origin;

    Graph graph;
    char* buffer;
    std::vector<int> compressRounds;
    std::vector<MCGAL::Halfedge*> seeds;
    std::vector<int> groupOffset;
    int dataOffset = 0;
    std::vector<int> schemaOffsets;
    std::vector<std::map<int, EncodeBoundarySchema>> boundaryIndexes;
    std::vector<int> currentLods;
    int offsetEnd = 0;
};

#endif