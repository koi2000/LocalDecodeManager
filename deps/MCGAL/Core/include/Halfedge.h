#ifndef HALFEDGE_H
#define HALFEDGE_H
#include <assert.h>
#include <atomic>
namespace MCGAL {
class Vertex;
class Facet;
class Halfedge {
    enum Flag { NotYetInQueue = 0, InQueue = 1, NoLongerInQueue = 2 };
    enum Flag2 { Original, Added, New };
    enum ProcessedFlag { NotProcessed, Processed };
    enum RemovedFlag { NotRemoved, Removed };
    enum BFSFlag { NotVisited, Visited };
    enum BoundaryFlag { NotBoundary, IsBoundary };

    Flag flag = NotYetInQueue;
    Flag2 flag2 = Original;
    ProcessedFlag processedFlag = NotProcessed;
    RemovedFlag removedFlag = NotRemoved;
    BFSFlag bfsFlag = NotVisited;
    BoundaryFlag boundaryFlag = NotBoundary;

  public:
    Halfedge(){};
    Vertex* vertex = nullptr;
    Vertex* end_vertex = nullptr;
    Facet* face = nullptr;
    Halfedge* next = nullptr;
    Halfedge* opposite = nullptr;
    Halfedge(Vertex* v1, Vertex* v2);
    ~Halfedge();
    unsigned long long horder = ~(unsigned long long)0;
    // std::atomic<unsigned long long> horder{(~(unsigned long long)0)};

    int poolId = -1;
    int indexInQueue = -1;
    int meshId = -1;
    int level = 0;
    int groupId = -1;

    // util
    void setMeshId(int meshId) {
        this->meshId = meshId;
    }
    void setVertex(Vertex* v1, Vertex* v2);
    void reset(Vertex* v1, Vertex* v2);

    inline void resetBfsFlag() {
        processedFlag = NotProcessed;
    }

    inline void resetState() {
        flag = NotYetInQueue;
        flag2 = Original;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        bfsFlag = NotVisited;
        horder = ~(unsigned long long)0;
        // horder.store(~(unsigned long long)0);
        level = 0;
        groupId = -1;
    }

    void setGroupId(int gid) {
        this->groupId = gid;
    }

    /* Flag 1 */

    inline void setInQueue() {
        flag = InQueue;
    }

    inline void removeFromQueue() {
        // assert(flag == InQueue);
        flag = NoLongerInQueue;
    }

    inline bool canAddInQueue() {
        return flag == NotYetInQueue;
    }

    /* Processed flag */

    inline void resetProcessedFlag() {
        processedFlag = NotProcessed;
    }

    inline void setProcessed() {
        processedFlag = Processed;
    }

    inline void setUnProcessed() {
        processedFlag = NotProcessed;
    }

    inline bool isBoundary() const {
        return boundaryFlag == IsBoundary;
    }

    inline void setBoundary() {
        boundaryFlag = IsBoundary;
    }

    inline void setNotBoundary() {
        boundaryFlag = NotBoundary;
    }

    inline bool isProcessed() const {
        return processedFlag == Processed;
    }

    /* Flag 2 */

    inline void setAdded() {
        assert(flag2 == Original);
        flag2 = Added;
    }

    inline void setNew() {
        assert(flag2 == Original);
        flag2 = New;
    }

    inline bool isAdded() const {
        return flag2 == Added;
    }

    inline bool isOriginal() const {
        return flag2 == Original;
    }
    inline void setOriginal() {
        flag2 = Original;
    }

    inline bool isNew() const {
        return flag2 == New;
    }

    /* Flag 3*/
    inline void setRemoved() {
        removedFlag = Removed;
    }

    inline bool isRemoved() {
        return removedFlag == Removed;
    }

    /* bfs Flag */
    inline void setVisited() {
        bfsFlag = Visited;
    }

    inline bool isVisited() {
        return bfsFlag == Visited;
    }
};
}  // namespace MCGAL

#endif