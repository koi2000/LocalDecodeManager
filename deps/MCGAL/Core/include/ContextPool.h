#ifndef CONTEXTPOOL_H
#define CONTEXTPOOL_H
#include "Configuration.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Vertex.h"
#include <vector>
namespace MCGAL {

class ContextPool {
  public:
    // try to use cuda zero copy
    MCGAL::Vertex* vpool = nullptr;
    MCGAL::Halfedge* hpool = nullptr;
    MCGAL::Facet* fpool = nullptr;
    int* vid2PoolId = nullptr;

    int* vindex;
    int* hindex;
    int* findex;
    int* vid;
    ContextPool();

  public:
    ~ContextPool();

    static ContextPool& getInstance() {
        static ContextPool contextPool;
        return contextPool;
    }

    void copyToCuda();
    void freeCuda();
    void mallocOnUnifiedMemory();
    void freeOnUnifiedMemory();

    void mallocOnCpu();
    void freeOnCpu();

    ContextPool(const ContextPool&) = delete;
    ContextPool& operator=(const ContextPool&) = delete;

    void reset() {
        freeOnCpu();
        mallocOnCpu();
    }

    int getFindex() {
        return *findex;
    }

    int getHindex() {
        return *hindex;
    }

    int getVindex() {
        return *vindex;
    }

    MCGAL::Vertex* getVertexByVid(int vid) {
        return &vpool[vid2PoolId[vid]];
    }

    MCGAL::Vertex* getVertexByIndex(int index) {
        return &vpool[index];
    }

    MCGAL::Halfedge* getHalfedgeByIndex(int index) {
        return &hpool[index];
    }

    MCGAL::Facet* getFacetByIndex(int index) {
        return &fpool[index];
    }

    inline MCGAL::Vertex* allocateVertexFromPool() {
        MCGAL::Vertex* v = &vpool[*vindex];
        v->id = *vid;
        vid2PoolId[(*vid)++] = (*vindex)++;
        return v;
    }

    inline MCGAL::Vertex* dupVertexFromPool(MCGAL::Vertex* v) {
        MCGAL::Vertex* vit = &vpool[*vindex];
        vit->setPoint(v->point());
        vit->id = *vid;
        vid2PoolId[(*vid)++] = (*vindex)++;
        return vit;
    }

    inline MCGAL::Vertex* allocateVertexFromPool(float x, float y, float z) {
        vpool[*vindex].setPoint({x, y, z, *vid});
        vid2PoolId[*vid] = *vindex;
        (*vid)++;
        return &vpool[(*vindex)++];
    }

    inline MCGAL::Vertex* allocateVertexFromPool(float x, float y, float z, int id) {
        vpool[*vindex].setPoint(x, y, z, id);
        vid2PoolId[id] = *vindex;
        return &vpool[(*vindex)++];
    }

    inline MCGAL::Vertex* allocateVertexFromPool(float x, float y, float z, int id, int meshId) {
        vpool[*vindex].setPoint(x, y, z, id);
        vpool[*vindex].setMeshId(meshId);
        vid2PoolId[(meshId << MESHID_OFFSET) + id] = *vindex;
        return &vpool[(*vindex)++];
    }

    inline MCGAL::Vertex* allocateVertexFromPool(MCGAL::Point p) {
        vpool[*vindex].setPoint(p);
        vid2PoolId[p.id] = *vindex;
        return &vpool[(*vindex)++];
    }

    inline MCGAL::Vertex* tryAllocVertexFromPool(MCGAL::Point p) {
        // 如果已经创建过这个点，就直接返回
        if (vid2PoolId[p.id] != -1) {
            return &vpool[vid2PoolId[p.id]];
        }
        vpool[*vindex].setPoint(p);
        vid2PoolId[p.id] = *vindex;
        return &vpool[(*vindex)++];
    }

    inline MCGAL::Vertex* allocateVertexFromPool(MCGAL::Point p, int meshId) {
        vpool[*vindex].setPoint(p);
        vpool[*vindex].setMeshId(meshId);
        vid2PoolId[(meshId << MESHID_OFFSET) + p.id] = *vindex;
        return &vpool[(*vindex)++];
    }

    inline MCGAL::Halfedge* allocateHalfedgeFromPool() {
        return &hpool[(*hindex)++];
    }

    inline MCGAL::Halfedge* tryAllocateHalfedgeFromPool(MCGAL::Vertex* v1, MCGAL::Vertex* v2) {
        for (MCGAL::Halfedge* hit : v2->halfedges) {
            if (hit->end_vertex == v1) {
                return hit;
            }
        }
        hpool[*hindex].reset(v1, v2);
        return &hpool[(*hindex)++];
    }

    inline MCGAL::Halfedge* allocateHalfedgeFromPool(MCGAL::Vertex* v1, MCGAL::Vertex* v2) {
        hpool[*hindex].reset(v1, v2);
        return &hpool[(*hindex)++];
    }

    inline MCGAL::Facet* allocateFaceFromPool() {
        return &fpool[(*findex)++];
    }

    inline MCGAL::Facet* allocateFaceFromPool(MCGAL::Halfedge* h) {
        fpool[*findex].reset(h);
        return &fpool[(*findex)++];
    }

    inline MCGAL::Facet* allocateFaceFromPool(std::vector<MCGAL::Vertex*> vts) {
        fpool[*findex].reset(vts);
        return &fpool[(*findex)++];
    }

    inline MCGAL::Facet* tryAllocateFaceFromPool(std::vector<MCGAL::Vertex*> vs) {
        Halfedge* prev = nullptr;
        Halfedge* head = nullptr;
        MCGAL::Facet* fit = allocateFaceFromPool();
        for (int i = 0; i < vs.size(); i++) {
            fit->vertices.push_back(vs[i]);
            Vertex* nextv = vs[(i + 1) % vs.size()];
            Halfedge* hf = tryAllocateHalfedgeFromPool(vs[i], nextv);
            fit->halfedges.push_back(hf);
            // vs[i]->halfedges.insert(hf);
            hf->face = fit;
            if (prev != NULL) {
                prev->next = hf;
            } else {
                head = hf;
            }
            if (i == vs.size() - 1) {
                hf->next = head;
            }
            prev = hf;
        }
        return fit;
    }

    inline int preAllocVertex(int size) {
        int ret = (*vindex);
        (*vindex) += size;
        return ret;
    }

    inline int preAllocHalfedge(int size) {
        int ret = (*hindex);
        (*hindex) += size;
        return ret;
    }

    inline int preAllocFace(int size) {
        int ret = (*findex);
        (*findex) += size;
        return ret;
    }
};

}  // namespace MCGAL
#endif