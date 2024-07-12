#ifndef LOCALDECODER_H
#define LOCALDECODER_H
#include "EncodeBoundarySchema.h"
#include "biops.h"
#include "core.h"
#include "syncbitops.h"
#include <cstring>
#include <map>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <vector>

/**
 * decode中要考虑的事情比encode要多
 * encode如果仅仅考虑边界周围仅有三角行的情况，则有较多的简化
 * 1. 周围均为三角形，
 * 2. 理想情况是两边同时进行阉割版的vertex split，不care对面在做什么，只关心自己做什么
 * 3. opposite可以为空，一个group里爱怎么玩怎么玩，影响不到其他人，duplicate一些点也没关系
 * ps: 不能duplicate 还是要在内存中只有一份
 * 存一份有点难搞，一个decode之后会影响周围的形状，暂时先不考虑
 */
class LocalDecoder {
  public:
    // 直接读取文件
    LocalDecoder(std::string path);

    void decode();

    void decodeOp(int groupId);

    void decodeOp(int groupId, int lod);

    void dumpToOFF(std::string path);

  private:
    std::vector<int> decodeFacetSymbolOp(int groupId, std::vector<MCGAL::Halfedge*>& boundarys);

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

    void readBoundaryCollapseMessage();

    void readSchemaOffset();

    void buildSchemaIndex(int lod);

    void decodeBoundary(int groupId, int lod, std::vector<MCGAL::Halfedge*> boundarys);

  private:
    MCGAL::Mesh mesh;
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