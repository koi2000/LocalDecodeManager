#include "./include/LocalSplitter.h"
#include <algorithm>
#include <map>

inline MCGAL::Halfedge* next_boundary(int groupId, MCGAL::Halfedge* boundary) {
    MCGAL::Halfedge* nxt = boundary->next;
    if (nxt->isBoundary()) {
        return nxt;
    }
    nxt = boundary->next->opposite->next;
    if (nxt->isBoundary()) {
        return nxt;
    }
    for (MCGAL::Halfedge* hit : boundary->end_vertex->halfedges) {
        if (hit->opposite != boundary && hit->isBoundary() && hit->face->groupId == groupId) {
            return hit;
        }
    }
}

LocalSplitter::LocalSplitter(std::string filename) {
    mesh = new MCGAL::Mesh();
    mesh->loadOFF(filename);
    markBoundry();
}

LocalSplitter::LocalSplitter(MCGAL::Mesh* mesh_, bool skipMarkBoundary) {
    this->mesh = mesh_;
    if (!skipMarkBoundary) {
        markBoundry();
    }
}

void LocalSplitter::loadMesh(MCGAL::Mesh* mesh_, bool skipMarkBoundary) {
    this->mesh = mesh_;
    if (!skipMarkBoundary) {
        markBoundry();
    }
}

std::vector<MCGAL::Halfedge*>& LocalSplitter::exportSeeds() {
    return seeds;
}

std::unordered_map<int, int>& LocalSplitter::exportDup2Origin() {
    return dup2origin;
}

std::unordered_map<int, std::unordered_map<int, int>>& LocalSplitter::exportOrigin2Dup() {
    return origin2dup;
}

Graph LocalSplitter::exportGraph() {
    return std::move(g);
}

std::vector<MCGAL::Mesh>& LocalSplitter::exportSubMeshes() {
    return subMeshes;
    // res.assign(std::make_move_iterator(subMeshes.begin()), std::make_move_iterator(subMeshes.end()));
}

/**
 * 使用copy_if直接拷贝过去
 * 对point需要进行dup
 */
void LocalSplitter::split(int groupNumber) {
    if (groupNumber != -1) {
        subMeshes.resize(groupNumber);
    }
    buildGraph();
#pragma omp parallel for
    for (int i = 0; i < subMeshes.size(); i++) {
        // subMeshes[i].faces.resize(mesh->size_of_facets());
        std::copy_if(mesh->faces.begin(), mesh->faces.end(), std::back_inserter(subMeshes[i].faces), [&i](const MCGAL::Facet* f) { return f->groupId == i; });
    }
#pragma omp parallel for
    for (size_t i = 0; i < subMeshes.size(); i++) {
        std::map<int, int> id2id;
        std::set<int> uniqueIds;
        for (MCGAL::Facet* fit : subMeshes[i].faces) {
            for (MCGAL::Halfedge* hit : fit->halfedges) {
                if (hit->isBoundary()) {
                    // 加入图中
                    MCGAL::Vertex* dup1 = nullptr;
                    MCGAL::Vertex* dup2 = nullptr;
                    if (id2id.count(hit->vertex->poolId)) {
                        dup1 = MCGAL::contextPool.getVertexByIndex(id2id[hit->vertex->poolId]);
                    } else {
#pragma omp critical
                        {
                            dup1 = MCGAL::contextPool.dupVertexFromPool(hit->vertex);
                            id2id[hit->vertex->poolId] = dup1->poolId;
                            uniqueIds.insert(dup1->poolId);
                            dup2origin[dup1->id] = hit->vertex->id;
                            origin2dup[hit->vertex->id].insert({i, dup1->id});
                        }
                    }

                    if (id2id.count(hit->end_vertex->poolId)) {
                        dup2 = MCGAL::contextPool.getVertexByIndex(id2id[hit->end_vertex->poolId]);
                    } else {
#pragma omp critical
                        {
                            dup2 = MCGAL::contextPool.dupVertexFromPool(hit->end_vertex);
                            id2id[hit->end_vertex->poolId] = dup2->poolId;
                            uniqueIds.insert(dup2->poolId);
                            dup2origin[dup2->id] = hit->end_vertex->id;
                            origin2dup[hit->end_vertex->id].insert({i, dup2->id});
                        }
                    }
                }
            }
        }
        for (MCGAL::Facet* fit : subMeshes[i].faces) {
            for (MCGAL::Halfedge* hit : fit->halfedges) {
                if (!id2id.count(hit->vertex->poolId)) {
                    uniqueIds.insert(hit->vertex->poolId);
                }
            }
        }
        // for (int id : uniqueIds) {
        //     subMeshes[i].vertices.push_back(MCGAL::contextPool.getVertexByIndex(id));
        // }
        std::transform(uniqueIds.begin(), uniqueIds.end(), std::back_inserter(subMeshes[i].vertices), [&](int id) {
            MCGAL::contextPool.getVertexByIndex(id)->setGroupId(i);
            return MCGAL::contextPool.getVertexByIndex(id);
        });
        // 加重了
        for (MCGAL::Facet* fit : subMeshes[i].faces) {
            for (MCGAL::Halfedge* hit : fit->halfedges) {
                MCGAL::Vertex* vit = hit->vertex;
                if (id2id.count(hit->vertex->poolId)) {
                    MCGAL::Vertex* dup = MCGAL::contextPool.getVertexByIndex(id2id[hit->vertex->poolId]);
                    if (dup->halfedges.empty()) {
                        for (MCGAL::Halfedge* ahit : hit->vertex->halfedges) {
                            if (ahit->face->groupId == i) {
                                dup->halfedges.push_back(ahit);
                            }
                        }
                    }

                    hit->vertex = dup;
                }
                /**
                 * v2 -> v1是boundary
                 * v2 -> v4是boundary
                 *   v1       _v3
                 *     \      /|
                 *     _\|   /
                 *        v2 ----> v4  
                */
                if (id2id.count(hit->end_vertex->poolId)) {
                    hit->end_vertex = MCGAL::contextPool.getVertexByIndex(id2id[hit->end_vertex->poolId]);
                }
            }
            for (int j = 0; j < fit->halfedges.size(); j++) {
                if (fit->halfedges[j]->isBoundary()) {
                    fit->halfedges[j]->opposite = nullptr;
                }
                fit->vertices[j] = fit->halfedges[j]->vertex;
            }
        }
    }
}

/***
 * 思路，找到所有的tripoint 作为起点，
 * 只有vertex为起始点的边才是有效边，通过该方法确定了方向
 * 从起始点出发，到另一个点为止，是一个边界，通过该方法可以构建图
 *
 */
void LocalSplitter::buildGraph() {
    std::set<MCGAL::Vertex*> triPoints;
    for (MCGAL::Vertex* vit : mesh->vertices) {
        std::set<int> cnt;
        for (MCGAL::Halfedge* hit : vit->halfedges) {
            cnt.insert(hit->face->groupId);
        }
        if (cnt.size() >= 3) {
            triPoints.insert(vit);
        }
    }
    for (MCGAL::Vertex* vit : triPoints) {
        for (MCGAL::Halfedge* hit : vit->halfedges) {
            if (hit->isBoundary()) {
                MCGAL::Halfedge* st = hit;
                int groupId = st->face->groupId;
                do {
                    assert(st->isBoundary());
                    st = next_boundary(groupId, st);
                } while (!triPoints.count(st->end_vertex));
                g.addEdge({hit->vertex->id, hit->end_vertex->id, hit->opposite->face->groupId, st->end_vertex->id}, {st->end_vertex->id, st->vertex->id, hit->face->groupId, hit->vertex->id});
            }
        }
    }
}

void LocalSplitter::dumpSubMesh(std::string path, int groupId) {
    assert(groupId < subMeshes.size());
    subMeshes[groupId].dumpto(path);
}

/**
 * 按照facet进行分割
 * mesh里只有vretex和facet，需要对vertex进行duplicate
 */
void LocalSplitter::markBoundry() {
    int fsize = mesh->size_of_facets();
    int sampleNumber = fsize / 3000;
    std::vector<int> stBfsId(fsize);
    std::vector<int> stVertex;
    for (int i = 0; i < stBfsId.size(); i++) {
        stBfsId[i] = i;
    }

    subMeshes.resize(sampleNumber);
    std::mt19937 g(RANDOM_SEED);
    std::shuffle(stBfsId.begin(), stBfsId.end(), g);
    stBfsId.resize(sampleNumber);
    std::queue<int> gateQueue;
    for (int i = 0; i < sampleNumber; i++) {
        seeds.push_back(mesh->faces[stBfsId[i]]->halfedges[0]);
        mesh->faces[stBfsId[i]]->halfedges[0]->face->groupId = i;
        gateQueue.push(mesh->faces[stBfsId[i]]->poolId);
        stVertex.push_back(mesh->faces[stBfsId[i]]->halfedges[0]->vertex->id);
        stVertex.push_back(mesh->faces[stBfsId[i]]->halfedges[0]->end_vertex->id);
    }

    while (!gateQueue.empty()) {
        int fid = gateQueue.front();
        gateQueue.pop();
        MCGAL::Facet* f = MCGAL::contextPool.getFacetByIndex(fid);
        if (f->isProcessed()) {
            continue;
        }
        f->setProcessedFlag();
        int vpoolId = -1;
        int minVid = -1;
        for (int j = 0; j < f->vertices.size(); j++) {
            if (f->vertices[j]->id > minVid) {
                minVid = f->vertices[j]->id;
                vpoolId = f->vertices[j]->poolId;
            }
        }
        MCGAL::Halfedge* hIt = nullptr;
        for (int j = 0; j < f->halfedges.size(); j++) {
            if (f->halfedges[j]->vertex->poolId == vpoolId) {
                hIt = f->halfedges[j];
                break;
            }
        }
        MCGAL::Halfedge* h = hIt;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite;
            if (!hOpp->face->isProcessed()) {
                hOpp->face->groupId = f->groupId;
                gateQueue.push(hOpp->face->poolId);
            }
            hIt = hIt->next;
        } while (hIt != h);
    }

    gateQueue.push(seeds[0]->poolId);
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(hid);
        gateQueue.pop();
        if (h->isProcessed()) {
            continue;
        }
        h->setProcessed();
        if (h->face->groupId != h->opposite->face->groupId) {
            h->setBoundary();
            h->opposite->setBoundary();
        }
        MCGAL::Halfedge* hIt = h->next;
        while (hIt->opposite != h) {
            if (!hIt->isProcessed()) {
                gateQueue.push(hIt->poolId);
            }
            hIt = hIt->opposite->next;
        }
    }
}