#include "./include/LocalSplitter.h"
#include <algorithm>
#include <map>

LocalSplitter::LocalSplitter(std::string filename) {
    mesh = new MCGAL::Mesh();
    mesh->loadOFF(filename);
    markBoundry();
}

LocalSplitter::LocalSplitter(MCGAL::Mesh* mesh_) {
    this->mesh = mesh_;
    markBoundry();
}

void LocalSplitter::loadMesh(MCGAL::Mesh* mesh_) {
    this->mesh = mesh_;
    markBoundry();
}

std::vector<MCGAL::Halfedge*>& LocalSplitter::exportSeeds() {
    return seeds;
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
void LocalSplitter::split() {
    for (int i = 0; i < subMeshes.size(); i++) {
        // subMeshes[i].faces.resize(mesh->size_of_facets());
        std::copy_if(mesh->faces.begin(), mesh->faces.end(), std::back_inserter(subMeshes[i].faces), [&i](const MCGAL::Facet* f) { return f->groupId == i; });
    }
    for (size_t i = 0; i < subMeshes.size(); i++) {
        std::map<int, int> id2id;
        std::set<int> uniqueIds;
        for (MCGAL::Facet* fit : subMeshes[i].faces) {
            for (MCGAL::Halfedge* hit : fit->halfedges) {
                if (hit->isBoundary()) {
                    // 加入图中
                    g.addEdge(hit->face->groupId, hit->opposite->face->groupId);
                    MCGAL::Vertex* dup1 = nullptr;
                    MCGAL::Vertex* dup2 = nullptr;
                    if (id2id.count(hit->vertex->poolId)) {
                        dup1 = MCGAL::contextPool.getVertexByIndex(id2id[hit->vertex->poolId]);
                    } else {
                        dup1 = MCGAL::contextPool.dupVertexFromPool(hit->vertex);
                        id2id[hit->vertex->poolId] = dup1->poolId;
                        // for (int j = 0; j < hit->vertex->halfedges.size(); j++) {
                        //     MCGAL::Halfedge* vhit = hit->vertex->halfedges[j];
                        //     if (vhit->face->groupId == i) {
                        //         dup1->halfedges.push_back(vhit);
                        //     }
                        // }
                        uniqueIds.insert(dup1->poolId);
                    }

                    if (id2id.count(hit->end_vertex->poolId)) {
                        dup2 = MCGAL::contextPool.getVertexByIndex(id2id[hit->end_vertex->poolId]);
                    } else {
                        dup2 = MCGAL::contextPool.dupVertexFromPool(hit->end_vertex);
                        id2id[hit->end_vertex->poolId] = dup2->poolId;
                        uniqueIds.insert(dup2->poolId);
                    }

                } else {
                    uniqueIds.insert(hit->vertex->poolId);
                }
            }
        }
        // for (int id : uniqueIds) {
        //     subMeshes[i].vertices.push_back(MCGAL::contextPool.getVertexByIndex(id));
        // }
        std::transform(uniqueIds.begin(), uniqueIds.end(), std::back_inserter(subMeshes[i].vertices), [&](int id) { return MCGAL::contextPool.getVertexByIndex(id); });
        for (MCGAL::Facet* fit : subMeshes[i].faces) {
            for (MCGAL::Halfedge* hit : fit->halfedges) {
                if (id2id.count(hit->vertex->poolId)) {
                    hit->vertex = MCGAL::contextPool.getVertexByIndex(id2id[hit->vertex->poolId]);
                    hit->vertex->halfedges.push_back(hit);
                }
                if (id2id.count(hit->end_vertex->poolId)) {
                    hit->end_vertex = MCGAL::contextPool.getVertexByIndex(id2id[hit->end_vertex->poolId]);
                    if (hit->opposite->face->groupId == i) {
                        hit->end_vertex->halfedges.push_back(hit->opposite);
                    }
                }
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