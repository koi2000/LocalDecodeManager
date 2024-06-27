#ifndef FACET_H
#define FACET_H
#include "Configuration.cuh"
#include "Halfedge.cuh"
#include "Vertex.cuh"
#include "stdint.h"
#include <vector>

namespace MCGAL {
class replacing_group;
class Mesh;
class Facet {
  public:
    typedef MCGAL::Point Point;
    enum Flag { Unknown = 0, Splittable = 1, Unsplittable = 2 };
    enum ProcessedFlag { NotProcessed, Processed };
    enum RemovedFlag { NotRemoved, Removed };
    enum VisitedFlag { NotVisited, Visited };

    Flag flag = Unknown;
    ProcessedFlag processedFlag = NotProcessed;
    RemovedFlag removedFlag = NotRemoved;
    VisitedFlag visitedFlag = NotVisited;
    Point removedVertexPos;

  public:
    int vertices[VERTEX_IN_FACE];
    int vertex_size = 0;
    // std::vector<Halfedge*> halfedges;
    int halfedges[HALFEDGE_IN_FACE];
    int halfedge_size = 0;
    int fid = -1;
    int poolId;
    int meshId = -1;
    int indexInQueue = -1;
    unsigned long long forder = ~(unsigned long long)0;

  public:
    ~Facet();

    // constructor
    Facet() {
        flag = Unknown;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        visitedFlag = NotVisited;
        forder = ~(unsigned long long)0;
    };
    Facet(const Facet& face);
    Facet(Halfedge* hit);
    Facet(std::vector<Vertex*>& vs);

    // pools
    void addHalfedge(Halfedge* halfedge);
    void addHalfedge(int halfedge);
    void addVertex(Vertex* vertex);
    void addVertex(int vertex);

    Vertex* getVertexByIndex(int index);
    Halfedge* getHalfedgeByIndex(int index);

    // utils
    Facet* clone();
    void reset(Halfedge* h);
    void reset(std::vector<Halfedge*>& hs);
    void reset(std::vector<Vertex*>& vs);
    void remove(Halfedge* h);
    int facet_degree();

    void setFid(int id) {
        fid = id;
    }

    void setMeshId(int id) {
        this->meshId = id;
    }

    __device__ void setMeshIdOnCuda(int id) {
        this->meshId = id;
    }

    // cuda
    __device__ Vertex* getVertexByIndexOnCuda(Vertex* vertices, int index);
    __device__ Halfedge* getHalfedgeByIndexOnCuda(Halfedge* halfedges, int index);
    __device__ void addHalfedgeOnCuda(Halfedge* halfedge);
    __device__ void addHalfedgeOnCuda(int halfedge);
    __device__ void addVertexOnCuda(Vertex* vertex);
    __device__ void addVertexOnCuda(int vertex);

    __device__ void resetOnCuda(Vertex* vertices, Halfedge* halfedges, Halfedge* h);

    __device__ void setRemovedOnCuda() {
        removedFlag = Removed;
    }

    __device__ inline bool isRemovedOnCuda() {
        return removedFlag == Removed;
    }

    __device__ inline void resetStateOnCuda() {
        flag = Unknown;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        visitedFlag = NotVisited;
        forder = ~(unsigned long long)0;
    }

    // override
    bool equal(const Facet& rhs) const;
    bool operator==(const Facet& rhs) const;

    // to_string method
    void print();
    void print_off();

    // set flags
    inline void resetState() {
        flag = Unknown;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        visitedFlag = NotVisited;
        forder = ~(unsigned long long)0;
    }

    inline void resetProcessedFlag() {
        processedFlag = NotProcessed;
    }

    inline bool isConquered() const {
        // return (flag == Splittable || flag == Unsplittable || removedFlag == Removed);
        return (flag == Splittable || flag == Unsplittable);
    }

    inline bool isSplittable() const {
        return (flag == Splittable);
    }

    __device__ inline bool isSplittableOnCuda() const {
        return (flag == Splittable);
    }

    inline bool isUnsplittable() const {
        return (flag == Unsplittable);
    }

    inline void setSplittable() {
        assert(flag == Unknown);
        flag = Splittable;
    }

    inline void setUnsplittable() {
        assert(flag == Unknown);
        flag = Unsplittable;
    }

    __device__ inline void setSplittableOnCuda() {
        assert(flag == Unknown);
        flag = Splittable;
    }

    __device__ inline void setUnsplittableOnCuda() {
        assert(flag == Unknown);
        flag = Unsplittable;
    }

    inline void setProcessedFlag() {
        processedFlag = Processed;
    }

    __device__ inline void setProcessedFlagOnCuda() {
        processedFlag = Processed;
    }

    inline bool isProcessed() const {
        return (processedFlag == Processed);
    }

    __device__ inline bool isProcessedOnCuda() const {
        return (processedFlag == Processed);
    }

    inline void setVisitedFlag() {
        visitedFlag = Visited;
    }

    inline bool isVisited() const {
        return (visitedFlag == Visited);
    }

    inline MCGAL::Point getRemovedVertexPos() {
        return removedVertexPos;
    }

    __device__ inline float* getRemovedVertexPosOnCuda() {
        return removedVertexPos.v;
    }

    inline void setRemovedVertexPos(Point p) {
        removedVertexPos = p;
    }

    /* Flag 3*/
    inline void setRemoved() {
        removedFlag = Removed;
    }

    inline bool isRemoved() {
        return removedFlag == Removed;
    }

    __device__ inline bool isConqueredOnCuda() const {
        // return (flag == Splittable || flag == Unsplittable || removedFlag == Removed);
        return (flag == Splittable || flag == Unsplittable);
    }

    __device__ inline void setRemovedVertexPosOnCuda(float* p) {
        removedVertexPos.v[0] = p[0];
        removedVertexPos.v[1] = p[1];
        removedVertexPos.v[2] = p[2];
    }

    __device__ inline void setRemovedVertexPosOnCuda(Point p) {
        removedVertexPos = p;
    }

  public:
    replacing_group* rg = NULL;
};
}  // namespace MCGAL
#endif