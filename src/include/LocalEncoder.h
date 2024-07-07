#ifndef LOCALENCODER_H
#define LOCALENCODER_H
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

#define BUFFER_SIZE 100 * 1024 * 1024
#define RANDOM_SEED 1035

class LocalEncoder {
  public:
    LocalEncoder(std::string filename);

    ~LocalEncoder();

    void encode();

    void dumpToFile(std::string filename);

  private:
    void dumpToBuffer();

    void writeBaseMesh();

    void dumpBoundaryToBuffer();

    void dumpFacetSymbolToBuffer();

    void dumpHalfedgeSymbolToBuffer();

    void dumpBoundryMergeMessageToBuffer(int prevDataOffset);

    bool encodeOp(int groupId);

    void encodeFacetSymbolOp(int groupId);

    void encodeHalfedgeSymbolOp(int groupId);

    void encodeLocalBoundary(int groupId, std::vector<int>& boundarys);

    bool boundaryRemovable(MCGAL::Halfedge* h);
    // 为网格设置边界
    void markBoundry();

    /**
     * mergeBoundary和splitBoundary中需要很多的设计
     * 目前想到的一个方案是在vertexRemove步骤中动态的去维护boundary
     * 顶点移除时投票选举新的groupId，并重新划定boundary，新增边一定不会是boundary
     * 分割的时候比较麻烦，需要记录一下面里哪个边可以是boundary
     */
    void mergeBoundary();

    void resetState();

    void resetBfsState();

    bool isRemovable(MCGAL::Vertex* v);

    MCGAL::Halfedge* vertexCut(std::queue<int>& gateQueue, MCGAL::Halfedge* startH);

    bool checkCompetition(MCGAL::Vertex* v) const;

    bool isConvex(const std::vector<MCGAL::Vertex*>& polygon) const;

    bool arePointsCoplanar(std::vector<MCGAL::Point>& points);

    bool areHalfedgesCoplanar(std::vector<MCGAL::Halfedge*>& halfedges);

    MCGAL::Point crossProduct(const MCGAL::Point& a, const MCGAL::Point& b);

    bool isPlanar(const std::vector<MCGAL::Vertex*>& polygon, float epsilon) const;

    bool willViolateManifold(const std::vector<MCGAL::Halfedge*>& polygon) const;

  private:
    MCGAL::Mesh mesh;
    char* buffer;
    // 多源点BFS起点
    std::vector<MCGAL::Halfedge*> seeds;
    // 每个group的压缩轮数
    std::vector<int> compressRounds;
    std::vector<int> groupOffset;
    int dataOffset = 0;
    // 数量为group数量
    std::vector<std::vector<std::deque<unsigned>>> connectFaceSym;
    std::vector<std::vector<std::deque<unsigned>>> connectEdgeSym;
    std::vector<std::vector<std::deque<MCGAL::Point>>> geometrySym;
    std::vector<std::deque<EncodeBoundarySchema>> boundarySchemas;
    std::vector<int> schemaSizes;
    // boundary halfedge
    std::vector<int> boundarys;
};

#endif