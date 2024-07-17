#ifndef PARTIAL_ENCODER_H
#define PARTIAL_ENCODER_H

#include "LocalSplitter.h"
#include "core.h"
#include <string>

class PartialEncoder {
  public:
    PartialEncoder(std::string path);

    void encode(int groupId, int round);

    void encodeInsideOp(int groupId);

    // void encodeInsideOp(int groupId);

    // void encodeBoundaryOp();

  private:
    void resetState(int groupId);

    bool isRemovable(MCGAL::Vertex* v);

    MCGAL::Halfedge* vertexCut(MCGAL::Mesh& mesh, std::set<int>& boundaryIds, std::queue<int>& gateQueue, MCGAL::Halfedge* startH);

    // void dumpToBuffer();

    // void writeBaseMesh();

    // void dumpBoundaryToBuffer();

    // void dumpFacetSymbolToBuffer();

    // void dumpHalfedgeSymbolToBuffer();

    // void dumpBoundryMergeMessageToBuffer(int prevDataOffset);

    // void encodeFacetSymbolOp(int groupId);

    // void encodeHalfedgeSymbolOp(int groupId);

    // void encodeLocalBoundary(int groupId, std::vector<int>& boundarys);

    // bool boundaryRemovable(MCGAL::Halfedge* h);
    // // 为网格设置边界
    // void markBoundry();

    // void mergeBoundary();

    // void resetBfsState();

    bool checkCompetition(MCGAL::Vertex* v) const;

    // bool isConvex(const std::vector<MCGAL::Vertex*>& polygon) const;

    bool arePointsCoplanar(std::vector<MCGAL::Point>& points);

    // bool areHalfedgesCoplanar(std::vector<MCGAL::Halfedge*>& halfedges);

    // MCGAL::Point crossProduct(const MCGAL::Point& a, const MCGAL::Point& b);

    bool isPlanar(const std::vector<MCGAL::Vertex*>& polygon, float epsilon) const;

    bool willViolateManifold(const std::vector<MCGAL::Halfedge*>& polygon) const;

  private:
    MCGAL::Mesh mesh;
    LocalSplitter splitter;
    std::vector<MCGAL::Mesh> subMeshes;
    std::vector<MCGAL::Halfedge*> seeds;
    // 每个group的压缩轮数
    std::vector<int> compressRounds;
};

#endif