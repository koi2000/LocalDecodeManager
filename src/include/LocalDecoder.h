#ifndef LOCALDECODER_H
#define LOCALDECODER_H
#include "biops.h"
#include "core.h"
#include "syncbitops.h"
#include <cstring>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>

class LocalDecoder {
  public:
    // 直接读取文件
    LocalDecoder(std::string path);

    void decode();

    void decodeOp(int groupId);

    void decodeOp(int groupId, int lod);

    void dumpToOFF(std::string path);

  private:
    std::vector<int> decodeFacetSymbolOp(int groupId);

    std::vector<int> decodeHalfedgeSymbolOp(int groupId);

    void insertRemovedVertex(std::vector<int>& fids);

    void joinFacet(std::vector<int>& hids);

    void loadBuffer(std::string path);

    void readBaseMesh();

    void buildFromBuffer(std::deque<MCGAL::Point>* p_pointDeque, std::deque<uint32_t*>* p_faceDeque);

    void readBoundary();

    void readMeta();

    void resetState();

     void resetBfsState();

  private:
    MCGAL::Mesh mesh;
    char* buffer;
    std::vector<int> compressRounds;
    std::vector<MCGAL::Halfedge*> seeds;
    std::vector<int> groupOffset;
    int dataOffset = 0;
};

#endif