#ifndef FACET_H
#define FACET_H
#include "Point.h"
#include <assert.h>
#include <vector>
namespace MCGAL {
class Point;
class Vertex;
class Halfedge;
class Facet {
  public:
    enum Flag { Unknown = 0, Splittable = 1, Unsplittable = 2 };
    enum ProcessedFlag { NotProcessed, Processed };
    enum RemovedFlag { NotRemoved, Removed };
    enum QueueFlag { NotYetInQueue = 0, InQueue = 1, NoLongerInQueue = 2 };
    QueueFlag queueFlag = NotYetInQueue;

    Flag flag = Unknown;
    ProcessedFlag processedFlag = NotProcessed;
    RemovedFlag removedFlag = NotRemoved;
    MCGAL::Point removedVertexPos;
    unsigned long long forder = ~(unsigned long long)0;
    int poolId = -1;
    int groupId = -1;
    int meshId = -1;
    int level = 0;

  public:
    std::vector<Vertex*> vertices;
    std::vector<Halfedge*> halfedges;
    int indexInQueue = -1;

  public:
    ~Facet();

    // constructor
    Facet(){};
    Facet(const Facet& face);
    Facet(Halfedge* hit);
    Facet(std::vector<Vertex*>& vs);
    // Facet(std::vector<Vertex*>& vs, Mesh* mesh);

    // utils
    void setMeshId(int meshId) {
        this->meshId = meshId;
    }

    void setGroupId(int gid) {
        this->groupId = gid;
    }
    Facet* clone();
    void reset(Halfedge* h);
    void reset(std::vector<Halfedge*>& hs);
    void reset(std::vector<Vertex*>& vs);
    // void reset(std::vector<Vertex*>& vs, Mesh* mesh);
    void remove(Halfedge* h);
    int facet_degree();

    // override
    bool equal(const Facet& rhs) const;
    bool operator==(const Facet& rhs) const;

    // to_string method
    void print();
    void print_off();

    inline void resetBfsFlag() {
        processedFlag = NotProcessed;
    }

    // set flags
    inline void resetState() {
        flag = Unknown;
        processedFlag = NotProcessed;
        removedFlag = NotRemoved;
        forder = ~(unsigned long long)0;
        // groupId = -1;
        level = 0;
        queueFlag = NotYetInQueue;
    }

    inline void setInQueue() {
        queueFlag = InQueue;
    }

    inline void removeFromQueue() {
        queueFlag = NoLongerInQueue;
    }

    inline bool isInQueue() {
        return queueFlag == InQueue;
    }

    inline void resetProcessedFlag() {
        processedFlag = NotProcessed;
    }

    inline bool isConquered() const {
        return (flag == Splittable || flag == Unsplittable || removedFlag == Removed);
    }

    inline bool isSplittable() const {
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
        assert(flag == Unknown || flag == Unsplittable);
        flag = Unsplittable;
    }

    inline void setProcessedFlag() {
        processedFlag = Processed;
    }

    inline void setUnProcessed() {
        processedFlag = NotProcessed;
    }

    inline bool isProcessed() const {
        return processedFlag == Processed;
    }

    Point getRemovedVertexPos() const;

    void setRemovedVertexPos(Point p);

    /* Flag 3*/
    inline void setRemoved() {
        removedFlag = Removed;
    }

    inline bool isRemoved() {
        return removedFlag == Removed;
    }
};
}  // namespace MCGAL

#endif