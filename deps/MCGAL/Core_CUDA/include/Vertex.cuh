#ifndef VERTEX_H
#define VERTEX_H

#include "Configuration.cuh"
#include <assert.h>
#include <cuda_runtime.h>
#include <stdexcept>
#include <stdio.h>
#include <unordered_set>
namespace MCGAL {

class Point {
  public:
    Point() {
        v[0] = 0.0;
        v[1] = 0.0;
        v[2] = 0.0;
    }

    Point(float x, float y, float z) {
        v[0] = x;
        v[1] = y;
        v[2] = z;
    }

    Point(Point* pt) {
        assert(pt);
        for (int i = 0; i < 3; i++) {
            v[i] = pt->v[i];
        }
    };

    float x() const {
        return v[0];
    }

    float y() const {
        return v[1];
    }

    float z() const {
        return v[2];
    }

    // Hash function for Point
    struct Hash {
        size_t operator()(const Point point) const {
            // Use a combination of hash functions for each member
            size_t hash = std::hash<float>{}(point.x());
            hash_combine(hash, std::hash<float>{}(point.y()));
            hash_combine(hash, std::hash<float>{}(point.z()));
            return hash;
        }
    };

    // Equality comparison for Vertex
    struct Equal {
        bool operator()(const Point p1, const Point p2) const {
            // Compare each member for equality
            return p1.x() == p2.x() && p1.y() == p2.y() && p1.z() == p2.z();
        }
    };

    static void hash_combine(size_t& seed, size_t hash) {
        seed ^= hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    float& operator[](int index) {
        if (index >= 0 && index < 3) {
            return v[index];
        } else {
            throw std::out_of_range("Index out of range");
        }
    }

  public:
    float v[3];
};

class Halfedge;
class Facet;
class Vertex;

class Vertex : public Point {
    enum Flag { Unconquered = 0, Conquered = 1 };
    Flag flag = Unconquered;
    unsigned int id = 0;

  public:
    Vertex() : Point() {
        // *lock = 1;
    }
    Vertex(const Point& p) : Point(p) {
        // *lock = 1;
    }
    Vertex(float v1, float v2, float v3) : Point(v1, v2, v3) {
        // *lock = 1;
    }

    void setMeshId(int id) {
        this->meshId = id;
    }

    __device__ void setMeshIdOnCuda(int id) {
        this->meshId = id;
    }

    int meshId = -1;
    int lock = 1;
    int poolId;
    int halfedges[HALFEDGE_IN_VERTEX];
    int halfedges_size = 0;
    // std::vector<Halfedge*> halfedges;

    void addHalfedge(Halfedge* halfedge);
    void addHalfedge(int halfedge);
    Halfedge* getHalfedgeByIndex(int index);
    void eraseHalfedgeByIndex(int index);
    void eraseHalfedgeByPointer(Halfedge* halfedge);
    inline void clearHalfedge() {
        halfedges_size = 0;
    }

    // cuda
    __device__ void addHalfedgeOnCuda(Halfedge* halfedge);
    __device__ void addHalfedgeOnCuda(int halfedge);

    __device__ void eraseHalfedgeByIndexOnCuda(int index);
    // __device__ void eraseHalfedgeByPointerOnCuda(Halfedge* halfedge);

    int vertex_degree() {
        return halfedges_size;
    }

    void print() {
        printf("%f %f %f\n", v[0], v[1], v[2]);
    }

    float x() const {
        return v[0];
    }

    float y() const {
        return v[1];
    }

    float z() const {
        return v[2];
    }

    Point point() {
        return Point(this);
    }

    void setPoint(const Point& p) {
        this->v[0] = p.x();
        this->v[1] = p.y();
        this->v[2] = p.z();
    }

    __device__ void setPointOnCuda(float* p) {
        this->v[0] = p[0];
        this->v[1] = p[1];
        this->v[2] = p[2];
    }

    inline void resetState() {
        flag = Unconquered;
    }

    inline bool isConquered() const {
        return flag == Conquered;
    }

    inline void setConquered() {
        flag = Conquered;
    }

    inline size_t getId() const {
        return id;
    }

    inline void setId(size_t nId) {
        id = nId;
    }
};
}  // namespace MCGAL
#endif