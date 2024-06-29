#include "./include/LocalEncoder.h"
#include "./include/BufferUtils.h"
#include <fstream>

inline bool cmpForder(MCGAL::Facet* f1, MCGAL::Facet* f2) {
    return f1->forder < f2->forder;
}

LocalEncoder::LocalEncoder(std::string filename) {
    buffer = new char[BUFFER_SIZE];
    mesh.loadOFF(filename);
}

LocalEncoder::~LocalEncoder() {
    if (buffer != NULL) {
        delete[] buffer;
    }
}

void LocalEncoder::dumpToFile(std::string path) {
    std::ofstream fout(path, std::ios::binary);
    int len = dataOffset;
    fout.write((char*)&len, sizeof(int));
    fout.write(buffer, len);
    fout.close();
}

void LocalEncoder::encode() {
    markBoundry();
    char path[256];
    sprintf(path, "./gisdata/decompressed_%d.mesh.off", -1);
    mesh.dumpto(path);
    resetState();
    for (int j = 0; j < 10; j++) {
        for (int i = 0; i < seeds.size(); i++) {
            int result = encodeOp(i);
        }
        mergeBoundary();
        resetState();
        char path[256];
        sprintf(path, "./gisdata/decompressed_%d.mesh.off", j);
        mesh.dumpto(path);
    }
    dumpToBuffer();
}

bool LocalEncoder::encodeOp(int groupId) {
    std::queue<int> gateQueue;
    gateQueue.push(seeds[groupId]->poolId);
    int removedCount = 0;
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(hid);
        gateQueue.pop();
        MCGAL::Facet* f = h->face;
        if (f->isConquered()) {
            continue;
        }
        bool hasRemovable = false;
        MCGAL::Halfedge* unconqueredVertexHE;
        for (MCGAL::Halfedge* hh = h->next; hh != h; hh = hh->next) {
            if (isRemovable(hh->end_vertex) && !hh->isBoundary() && !hh->opposite->isBoundary()) {
                hasRemovable = true;
                unconqueredVertexHE = hh;
                break;
            }
        }
        if (!hasRemovable) {
            f->setUnsplittable();
            MCGAL::Halfedge* hh = h;
            do {
                hh->vertex->setConquered();
                MCGAL::Halfedge* hOpp = hh->opposite;
                if (!hOpp->face->isConquered() && !hOpp->isBoundary() && !h->isBoundary()) {
                    gateQueue.push(hOpp->poolId);
                }
            } while ((hh = hh->next) != h);
        } else {
            removedCount++;
            vertexCut(gateQueue, unconqueredVertexHE);
        }
    }
    if (removedCount != 0) {
        resetBfsState();
        compressRounds[groupId]++;
        encodeFacetSymbolOp(groupId);
        resetBfsState();
        encodeHalfedgeSymbolOp(groupId);
        resetBfsState();
        // encodeLocalBoundary(groupId, boundarys);
    }
    return removedCount != 0;
}

void LocalEncoder::encodeFacetSymbolOp(int groupId) {
    std::deque<unsigned> facetSym;
    std::deque<MCGAL::Point> points;
    std::queue<int> gateQueue;
    gateQueue.push(seeds[groupId]->face->poolId);
    while (!gateQueue.empty()) {
        int fid = gateQueue.front();
        MCGAL::Facet* f = MCGAL::contextPool.getFacetByIndex(fid);
        gateQueue.pop();
        if (f->isProcessed()) {
            continue;
        }
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
        unsigned sym = f->isSplittable();
        facetSym.push_back(sym);

        if (sym) {
            MCGAL::Point rmved = f->getRemovedVertexPos();
            points.push_back(rmved);
        }

        f->setProcessedFlag();
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite;
            // 对方没有被处理，且该边不是边界
            if (!hOpp->face->isProcessed() && !hOpp->isBoundary()) {
                gateQueue.push(hOpp->face->poolId);
            }
            hIt = hIt->next;
        } while (hIt != h);
    }
    connectFaceSym[groupId].push_back(facetSym);
    geometrySym[groupId].push_back(points);
}

void LocalEncoder::encodeHalfedgeSymbolOp(int groupId) {
    std::deque<unsigned> halfedgeSym;
    std::queue<int> gateQueue;
    gateQueue.push(seeds[groupId]->face->poolId);
    while (!gateQueue.empty()) {
        int fid = gateQueue.front();
        MCGAL::Facet* f = MCGAL::contextPool.getFacetByIndex(fid);
        gateQueue.pop();
        if (f->isProcessed()) {
            continue;
        }
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
        f->setProcessedFlag();
        do {
            unsigned sym;
            if (hIt->isOriginal())
                sym = 0;
            else
                sym = 1;
            halfedgeSym.push_back(sym);
            MCGAL::Halfedge* hOpp = hIt->opposite;
            // 对方没有被处理，且该边不是边界
            if (!hOpp->face->isProcessed() && !hOpp->isBoundary()) {
                gateQueue.push(hOpp->face->poolId);
            }
            hIt = hIt->next;
        } while (hIt != h);
    }
    connectEdgeSym[groupId].push_back(halfedgeSym);
}

/**
 * 对所有点集维护一个bitmap
 */
void LocalEncoder::encodeLocalBoundary(int groupId, std::vector<int>& boundarys) {
    std::deque<std::pair<int, int>> vidq;
    std::deque<MCGAL::Point> pointq;
    for (int i = 0; i < boundarys.size(); i++) {
        MCGAL::Halfedge* hit = MCGAL::contextPool.getHalfedgeByIndex(boundarys[i]);
        if (hit->opposite->groupId < groupId) {
            bool res = boundaryPointRemovable(hit->vertex);
            if (res) {
                MCGAL::Halfedge* newh = mesh.erase_center_vertex(hit);
                newh->setGroupId(groupId);
                // 移除中心顶点后需要重新划定边界
                for (MCGAL::Halfedge* h : newh->face->halfedges) {
                    if (h->face->groupId != h->opposite->face->groupId) {
                        h->setBoundary();
                        h->opposite->setBoundary();
                    }
                }
                // 需要保存以下信息，并在内存中构建一个index
                pointq.push_back(hit->vertex);
                vidq.push_back({newh->vertex->id, newh->end_vertex->id});
            }
        }
    }
    boundaryPoints.push_back(pointq);
    boundaryVidPair.push_back(vidq);
}

void LocalEncoder::dumpToBuffer() {
    writeBaseMesh();
    // 写入起始点
    writeInt(buffer, dataOffset, seeds.size());
    for (int i = 0; i < seeds.size(); i++) {
        writeInt(buffer, dataOffset, seeds[i]->vertex->id);
        writeInt(buffer, dataOffset, seeds[i]->end_vertex->id);
    }
    for (int i = 0; i < compressRounds.size(); i++) {
        writeInt(buffer, dataOffset, compressRounds[i]);
    }
    resetBfsState();
    dumpBoundaryToBuffer();
    dumpBoundryMergeMessageToBuffer();
    dumpFacetSymbolToBuffer();
    // dumpHalfedgeSymbolToBuffer();
}

void LocalEncoder::dumpBoundaryToBuffer() {
    std::queue<int> gateQueue;
    gateQueue.push(seeds[0]->poolId);
    while (!gateQueue.empty()) {
        int hid = gateQueue.front();
        MCGAL::Halfedge* h = MCGAL::contextPool.getHalfedgeByIndex(hid);
        gateQueue.pop();
        if (h->isProcessed()) {
            continue;
        }
        h->setProcessed();
        MCGAL::Halfedge* hIt = h->next;
        while (hIt->opposite != h) {
            if (!hIt->isProcessed()) {
                gateQueue.push(hIt->poolId);
            }
            hIt = hIt->opposite->next;
        }
        unsigned sym;
        if (h->isBoundary()) {
            sym = 1;
        } else {
            sym = 0;
        }
        writeChar(buffer, dataOffset, sym);
    }
}

void LocalEncoder::mergeBoundary() {
    std::deque<std::pair<int, int>> vidq;
    std::deque<MCGAL::Point> pointq;
    for (int i = 0; i < boundarys.size(); i++) {
        MCGAL::Halfedge* hit = MCGAL::contextPool.getHalfedgeByIndex(boundarys[i]);
        if (hit->opposite->face->groupId > hit->face->groupId) {
            bool res = boundaryPointRemovable(hit->vertex);
            if (res) {
                MCGAL::Halfedge* newh = mesh.erase_center_vertex(hit);
                newh->setGroupId(hit->face->groupId);
                // 移除中心顶点后需要重新划定边界
                for (MCGAL::Halfedge* h : newh->face->halfedges) {
                    if (h->face->groupId != h->opposite->face->groupId) {
                        if (!h->isBoundary()) {
                            boundarys.push_back(h->poolId);
                            h->setBoundary();
                            h->opposite->setBoundary();
                        } else {
                            
                        }
                    }
                }
                // 需要保存以下信息，并在内存中构建一个index
                pointq.push_back(hit->vertex);
                vidq.push_back({newh->vertex->id, newh->end_vertex->id});
            }
        }
    }
    boundaryPoints.push_back(pointq);
    boundaryVidPair.push_back(vidq);
    auto boundaryEnd = std::remove_if(boundarys.begin(), boundarys.end(), [](int hid) {
        MCGAL::Halfedge* hit = MCGAL::contextPool.getHalfedgeByIndex(hid);
        return hit->isRemoved();
    });
    boundarys.resize(std::distance(boundarys.begin(), boundaryEnd));
    // fix boundary
}

void LocalEncoder::dumpBoundryMergeMessageToBuffer() {
    std::vector<int> offsets(boundaryPoints.size());
    for (int i = 0; i < boundaryPoints.size(); i++) {
        offsets[i] = boundaryPoints[i].size();
    }
    for (int i = 0; i < boundaryPoints.size(); i++) {
        writeInt(buffer, dataOffset, offsets[i]);
    }
    for (int i = 0; i < boundaryPoints.size(); i++) {
        for (int j = 0; j < boundaryPoints[i].size(); j++) {
            writeInt(buffer, dataOffset, boundaryVidPair[i][j].first);
            writeInt(buffer, dataOffset, boundaryVidPair[i][j].second);
            writePoint(buffer, dataOffset, boundaryPoints[i][j]);
        }
    }
}

void LocalEncoder::dumpFacetSymbolToBuffer() {
    std::vector<int> offsets(seeds.size());
    offsets[0] = 0;
    for (int i = 0; i < seeds.size() - 1; i++) {
        int sum = 0;
        for (int j = 0; j < connectFaceSym[i].size(); j++) {
            sum += connectFaceSym[i][j].size() + geometrySym[i][j].size() * sizeof(int) * 4 + connectEdgeSym[i][j].size();
        }
        offsets[i + 1] = sum;
        // offsets[i + 1] = connectFaceSym[i].size() + geometrySym[i].size() * sizeof(int) * 4;
    }
    for (int i = 1; i < seeds.size(); i++) {
        offsets[i] += offsets[i - 1];
    }
    for (int i = 0; i < seeds.size(); i++) {
        offsets[i] += (dataOffset + offsets.size() * sizeof(int));
    }
    for (int i = 0; i < offsets.size(); i++) {
        writeInt(buffer, dataOffset, offsets[i]);
    }

    for (int i = 0; i < seeds.size(); i++) {
        for (int k = connectFaceSym[i].size() - 1; k >= 0; k--) {
            std::deque<unsigned> connFSym = connectFaceSym[i][k];
            std::deque<unsigned> connESym = connectEdgeSym[i][k];
            std::deque<MCGAL::Point> geomSym = geometrySym[i][k];
            int index = 0;
            for (unsigned j = 0; j < connFSym.size(); ++j) {
                unsigned sym = connFSym[j];
                writeChar(buffer, dataOffset, sym);
                if (sym) {
                    writePoint(buffer, dataOffset, geomSym[index++]);
                }
            }
            for (unsigned j = 0; j < connESym.size(); ++j) {
                unsigned sym = connESym[j];
                writeChar(buffer, dataOffset, sym);
            }
        }
    }
}

void LocalEncoder::dumpHalfedgeSymbolToBuffer() {
    // std::vector<int> offsets(seeds.size());
    // offsets[0] = 0;
    // for (int i = 0; i < seeds.size(); i++) {
    //     offsets[i + 1] = connectEdgeSym[i].size();
    // }
    // for (int i = 1; i < seeds.size(); i++) {
    //     offsets[i] += offsets[i - 1];
    // }
    // for (int i = 0; i < seeds.size(); i++) {
    //     offsets[i] += (dataOffset + offsets.size() * sizeof(int));
    // }
    // for (int i = 0; i < offsets.size(); i++) {
    //     writeInt(buffer, dataOffset, offsets[i]);
    // }
    for (int i = 0; i < seeds.size(); i++) {
        for (int k = connectEdgeSym[i].size() - 1; k >= 0; k--) {
            std::deque<unsigned> connSym = connectEdgeSym[i][k];
            for (unsigned j = 0; j < connSym.size(); ++j) {
                unsigned sym = connSym[j];
                writeChar(buffer, dataOffset, sym);
            }
        }
    }
}

MCGAL::Halfedge* LocalEncoder::vertexCut(std::queue<int>& gateQueue, MCGAL::Halfedge* startH) {
    MCGAL::Vertex* v = startH->end_vertex;
    assert(!v->isConquered());
    assert(v->vertex_degree() > 2);

    MCGAL::Halfedge* h = startH->opposite;
    MCGAL::Halfedge* end(h);
    int removed = 0;
    do {
        MCGAL::Facet* f = h->face;
        assert(!f->isConquered() && !f->isRemoved());

        if (f->facet_degree() > 3) {
            MCGAL::Halfedge* hSplit(h->next);
            for (; hSplit->next->next != h; hSplit = hSplit->next)
                ;
            MCGAL::Halfedge* hCorner = mesh.split_facet(h, hSplit);
            hCorner->setAdded();
        }
        h->end_vertex->setConquered();
        removed++;
    } while ((h = h->opposite->next) != end);
    MCGAL::Point vPos = startH->end_vertex->point();
    MCGAL::Halfedge* hNewFace = mesh.erase_center_vertex(startH);
    MCGAL::Facet* added_face = hNewFace->face;
    added_face->setSplittable();
    added_face->setRemovedVertexPos(vPos);
    h = hNewFace;
    do {
        MCGAL::Halfedge* hOpp = h->opposite;
        if (hOpp == nullptr) {
            continue;
        }
        if (!hOpp->face->isConquered() && !hOpp->isBoundary() && !h->isBoundary()) {
            gateQueue.push(hOpp->poolId);
        }
    } while ((h = h->next) != hNewFace);
    return hNewFace;
}

void LocalEncoder::resetBfsState() {
    for (auto fit = mesh.faces.begin(); fit != mesh.faces.end();) {
        if ((*fit)->isRemoved()) {
            fit = mesh.faces.erase(fit);
        } else {
            (*fit)->resetBfsFlag();
            for (MCGAL::Halfedge* hit : (*fit)->halfedges) {
                hit->resetBfsFlag();
            }
            fit++;
        }
    }
}

void LocalEncoder::resetState() {
    for (auto vit = mesh.vertices.begin(); vit != mesh.vertices.end();) {
        if ((*vit)->isRemoved()) {
            vit = mesh.vertices.erase(vit);
        } else {
            (*vit)->resetState();
            vit++;
        }
    }
    for (auto fit = mesh.faces.begin(); fit != mesh.faces.end();) {
        if ((*fit)->isRemoved()) {
            fit = mesh.faces.erase(fit);
        } else {
            (*fit)->resetState();
            for (MCGAL::Halfedge* hit : (*fit)->halfedges) {
                hit->resetState();
            }
            fit++;
        }
    }
}

void LocalEncoder::markBoundry() {
    int fsize = mesh.size_of_facets();
    int sampleNumber = fsize / 3000;
    connectFaceSym.resize(sampleNumber);
    connectEdgeSym.resize(sampleNumber);
    geometrySym.resize(sampleNumber);
    compressRounds = std::vector<int>(sampleNumber, 0);
    std::vector<int> stBfsId(fsize);
    std::vector<int> stVertex;
    for (int i = 0; i < stBfsId.size(); i++) {
        stBfsId[i] = i;
    }
    std::mt19937 g(RANDOM_SEED);
    std::shuffle(stBfsId.begin(), stBfsId.end(), g);
    stBfsId.resize(sampleNumber);
    std::queue<int> gateQueue;
    for (int i = 0; i < sampleNumber; i++) {
        seeds.push_back(mesh.faces[stBfsId[i]]->halfedges[0]);
        mesh.faces[stBfsId[i]]->halfedges[0]->face->groupId = i;
        gateQueue.push(mesh.faces[stBfsId[i]]->poolId);
        stVertex.push_back(mesh.faces[stBfsId[i]]->halfedges[0]->vertex->id);
        stVertex.push_back(mesh.faces[stBfsId[i]]->halfedges[0]->end_vertex->id);
    }

    // for (int i = 0; i < sampleNumber; i++) {
    //     int poolId1 = MCGAL::contextPool.vid2PoolId[stVertex[i * 2]];
    //     int poolId2 = MCGAL::contextPool.vid2PoolId[stVertex[i * 2 + 1]];
    //     MCGAL::Vertex* v1 = MCGAL::contextPool.getVertexByIndex(poolId1);
    //     MCGAL::Vertex* v2 = MCGAL::contextPool.getVertexByIndex(poolId2);
    //     for (int j = 0; j < v1->halfedges.size(); j++) {
    //         if (v1->halfedges[j]->end_vertex == v2) {
    //             v1->halfedges[j]->face->groupId = i;
    //             gateQueue.push(v1->halfedges[j]->face->poolId);
    //             break;
    //         }
    //     }
    // }

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
            boundarys.push_back(h->poolId);
            boundarys.push_back(h->opposite->poolId);
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

bool LocalEncoder::isRemovable(MCGAL::Vertex* v) {
    for (int i = 0; i < seeds.size(); i++) {
        if (v->poolId == seeds[i]->vertex->poolId || v->poolId == seeds[i]->end_vertex->poolId) {
            return false;
        }
    }

    if (!v->isConquered() && v->vertex_degree() > 2 && v->vertex_degree() <= 8) {
        std::vector<MCGAL::Point> vh_oneRing;
        std::vector<MCGAL::Halfedge*> heh_oneRing;
        heh_oneRing.reserve(v->vertex_degree());
        for (MCGAL::Halfedge* hit : v->halfedges) {
            if (hit->isBoundary() || hit->opposite->isBoundary()) {
                return false;
            }
            vh_oneRing.push_back(hit->opposite->vertex->point());
            heh_oneRing.push_back(hit);
        }
        bool removable = !willViolateManifold(heh_oneRing);  // && arePointsCoplanar(vh_oneRing);
        // if (removable) {
        //     return checkCompetition(v);
        // }
        return removable;
    }
    return false;
}

bool LocalEncoder::boundaryPointRemovable(MCGAL::Vertex* v) {
    bool res = false;
    for (MCGAL::Halfedge* hit : v->halfedges) {
        if (hit->face->facet_degree() != 3) {
            return false;
        }
    }
    if (v->vertex_degree() > 2 && v->vertex_degree() <= 8) {
        std::vector<MCGAL::Point> vh_oneRing;
        std::vector<MCGAL::Halfedge*> heh_oneRing;
        heh_oneRing.reserve(v->vertex_degree());
        for (MCGAL::Halfedge* hit : v->halfedges) {
            vh_oneRing.push_back(hit->opposite->vertex->point());
            heh_oneRing.push_back(hit);
        }
        bool removable = !willViolateManifold(heh_oneRing);  // && arePointsCoplanar(vh_oneRing);
        // if (removable) {
        //     return checkCompetition(v);
        // }
        return removable;
    }
    return false;
}

MCGAL::Point LocalEncoder::crossProduct(const MCGAL::Point& a, const MCGAL::Point& b) {
    MCGAL::Point result;
    result.v[0] = a.y() * b.z() - a.z() * b.y();
    result.v[1] = a.z() * b.x() - a.x() * b.z();
    result.v[2] = a.x() * b.y() - a.y() * b.x();
    return result;
}

bool LocalEncoder::arePointsCoplanar(std::vector<MCGAL::Point>& points) {
    if (points.size() < 3) {
        std::cerr << "Error: Insufficient points to form a plane." << std::endl;
        return false;
    }
    MCGAL::Point v1 = {points[1].x() - points[0].x(), points[1].y() - points[0].y(), points[1].z() - points[0].z()};
    MCGAL::Point v2 = {points[2].x() - points[0].x(), points[2].y() - points[0].y(), points[2].z() - points[0].z()};
    MCGAL::Point normal = crossProduct(v1, v2);

    for (size_t i = 3; i < points.size(); ++i) {
        MCGAL::Point vecToPoint = {points[i].x() - points[0].x(), points[i].y() - points[0].y(), points[i].z() - points[0].z()};
        MCGAL::Point cross = crossProduct(vecToPoint, normal);
        if (fabs(cross.x()) > 0.000001 || fabs(cross.y()) > 0.000001 || fabs(cross.z()) > 0.000001) {
            return false;
        }
    }
    return true;
}

bool LocalEncoder::checkCompetition(MCGAL::Vertex* v) const {
    std::set<MCGAL::Facet*> fset;
    for (int i = 0; i < v->halfedges.size(); i++) {
        MCGAL::Halfedge* hit = v->halfedges[i];
        fset.insert(hit->face);
    }
    for (MCGAL::Facet* fit : fset) {
        for (int i = 0; i < fit->halfedges.size(); i++) {
            if (fit->halfedges[i]->isAdded() || fit->halfedges[i]->opposite->isAdded()) {
                return false;
            }
        }
    }
    return true;
}

bool LocalEncoder::willViolateManifold(const std::vector<MCGAL::Halfedge*>& polygon) const {
    unsigned i_degree = polygon.size();
    for (unsigned i = 0; i < i_degree; ++i) {
        MCGAL::Halfedge* it = polygon[i];
        if (it->face->facet_degree() == 3) {
            continue;
        }

        for (int j = 0; j < i_degree; j++) {
            MCGAL::Halfedge* jt = polygon[j];
            if (i == j)
                continue;
            if (it->face == jt->opposite->face) {
                for (int k = 0; k < it->end_vertex->halfedges.size(); k++) {
                    if (it->end_vertex->halfedges[k]->end_vertex == jt->end_vertex) {
                        return true;
                    }
                }
            } else if (it->face == jt->face) {
                for (int k = 0; k < it->end_vertex->halfedges.size(); k++) {
                    if (it->end_vertex->halfedges[k]->end_vertex == jt->end_vertex) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void LocalEncoder::writeBaseMesh() {
    unsigned i_nbVerticesBaseMesh = mesh.size_of_vertices();
    unsigned i_nbFacesBaseMesh = mesh.size_of_facets();

    writeInt(buffer, dataOffset, i_nbVerticesBaseMesh);
    writeInt(buffer, dataOffset, i_nbFacesBaseMesh);
    int id = 0;
    for (MCGAL::Vertex* vit : mesh.vertices) {
        MCGAL::Point point = vit->point();
        writePoint(buffer, dataOffset, point);
        vit->setVid(id++);
    }
    for (MCGAL::Facet* fit : mesh.faces) {
        unsigned i_faceDegree = fit->facet_degree();
        writeInt(buffer, dataOffset, i_faceDegree);
        for (MCGAL::Halfedge* hit : fit->halfedges) {
            writeInt(buffer, dataOffset, hit->vertex->getVid());
        }
    }
}