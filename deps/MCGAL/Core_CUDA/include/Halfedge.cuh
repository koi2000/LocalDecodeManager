#ifndef HALFEDGE_H
#define HALFEDGE_H
#include <assert.h>
#include <cuda_runtime.h>
namespace MCGAL {
class Vertex;
class Facet;
class Halfedge {
    enum Flag { NotYetInQueue = 0, InQueue = 1, NoLongerInQueue = 2 };
    enum Flag2 { Original, Added, New };
    enum ProcessedFlag { NotProcessed, Processed };
    enum RemovedFlag { NotRemoved, Removed };
    enum BFSFlag { NotVisited, Visited };

    Flag flag = NotYetInQueue;
    Flag2 flag2 = Original;
    ProcessedFlag processedFlag = NotProcessed;
    RemovedFlag removedFlag = NotRemoved;
    BFSFlag bfsFlag = NotVisited;

  public:
    int hid = -1;
    int meshId = -1;
    int lock = 0;
    int poolId;
    int vertex_ = -1;
    int end_vertex_ = -1;
    int facet_ = -1;
    int next_ = -1;
    int opposite_ = -1;
    int count = 0;
    int parent = -1;
    unsigned long long horder = ~(unsigned long long)0;
    Halfedge() {
        flag = NotYetInQueue;
        flag2 = Original;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        bfsFlag = NotVisited;
        horder = ~(unsigned long long)0;
        parent = -1;
    };
    // Vertex* vertex = nullptr;
    // Vertex* end_vertex = nullptr;
    // Facet* face = nullptr;
    // Halfedge* next = nullptr;
    // Halfedge* opposite = nullptr;
    Halfedge(Vertex* v1, Vertex* v2);
    Halfedge(int v1, int v2);
    ~Halfedge();

    Vertex* vertex();
    Vertex* end_vertex();
    Facet* facet();
    Halfedge* opposite();
    Halfedge* next();

    // cpu
    void setOpposite(Halfedge* opposite);
    void setOpposite(int opposite);

    void setNext(Halfedge* next);
    void setNext(int next);

    void setFacet(Facet* facet);
    void setFacet(int facet);

    void reset(Vertex* v1, Vertex* v2);

    void setHid(int id) {
        hid = id;
    }

    void setMeshId(int id) {
        this->meshId = id;
    }

    __device__ void setMeshIdOnCuda(int id) {
        this->meshId = id;
    }

    // gpu
    __device__ void setOppositeOnCuda(Halfedge* opposite);
    __device__ void setOppositeOnCuda(int opposite);

    __device__ void setNextOnCuda(Halfedge* next);
    __device__ void setNextOnCuda(int next);

    __device__ void setFacetOnCuda(Facet* facet);
    __device__ void setFacetOnCuda(int facet);
    // __device__ void resetOnCuda(Vertex* v1, Vertex* v2);

    __device__ Vertex* dvertex(Vertex* vertices);
    __device__ Vertex* dend_vertex(Vertex* vertices);
    __device__ Facet* dfacet(Facet* facets);
    __device__ Halfedge* dopposite(Halfedge* halfedges);
    __device__ Halfedge* dnext(Halfedge* halfedges);
    __device__ void resetOnCuda(Vertex* v1, Vertex* v2);
    __device__ void resetOnCuda(int v1, int v2);

    __device__ inline void resetStateOnCuda() {
        flag = NotYetInQueue;
        flag2 = Original;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        bfsFlag = NotVisited;
        horder = ~(unsigned long long)0;
        parent = -1;
    }

    inline void resetState() {
        flag = NotYetInQueue;
        flag2 = Original;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        bfsFlag = NotVisited;
        horder = ~(unsigned long long)0;
        parent = -1;
    }

    /* Flag 1 */

    inline void setInQueue() {
        flag = InQueue;
    }

    inline void removeFromQueue() {
        assert(flag == InQueue);
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

    inline bool isProcessed() const {
        return (processedFlag == Processed);
    }

    __device__ inline bool isProcessedOnCuda() const {
        return (processedFlag == Processed);
    }

    /* Flag 2 */

    inline void setAdded() {
        assert(flag2 == Original);
        flag2 = Added;
    }

    __device__ inline void setAddedOnCuda() {
        assert(flag2 == Original);
        flag2 = Added;
    }

    __device__ inline void setOriginalOnCuda() {
        flag2 = Original;
    }

    inline void setNew() {
        assert(flag2 == Original);
        flag2 = New;
    }

    inline bool isAdded() const {
        return flag2 == Added;
    }

    __device__ inline bool isAddedOnCuda() const {
        return flag2 == Added;
    }

    inline bool isOriginal() const {
        return flag2 == Original;
    }

    inline bool isNew() const {
        return flag2 == New;
    }

    /* Flag 3*/
    __host__ __device__ inline void setRemoved() {
        removedFlag = Removed;
    }

    inline bool isRemoved() {
        return removedFlag == Removed;
    }

    __device__ inline bool isRemovedOnCuda() {
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