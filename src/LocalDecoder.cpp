#include "./include/LocalDecoder.h"
#include "./include/BufferUtils.h"
#include <fstream>

LocalDecoder::LocalDecoder(std::string path) {
    loadBuffer(path);
    readBaseMesh();
    readMeta();
}

void LocalDecoder::decode() {}

void LocalDecoder::decodeOp(int groupId) {}

void LocalDecoder::decodeOp(int groupId, int lod) {
    // if (compressRounds[groupId] < lod) {
    //     std::cout << "out of boundary" << std::endl;
    // }
    int count = 0;
    while (count < lod) {
        std::vector<int> fids = decodeFacetSymbolOp(groupId);
        resetBfsState();
        std::vector<int> hids = decodeHalfedgeSymbolOp(groupId);
        insertRemovedVertex(fids);
        joinFacet(hids);
        resetState();
        count++;
    }
}

void LocalDecoder::dumpToOFF(std::string path) {
    mesh.dumpto(path);
}

void LocalDecoder::insertRemovedVertex(std::vector<int>& fids) {
    for (int i = 0; i < fids.size(); i++) {
        MCGAL::Facet* fit = MCGAL::contextPool.getFacetByIndex(fids[i]);

        MCGAL::Halfedge* hit = mesh.create_center_vertex(fit->halfedges[0]);
        hit->vertex->setPoint(fit->getRemovedVertexPos());
    }
}

void LocalDecoder::joinFacet(std::vector<int>& hids) {
    for (int i = 0; i < hids.size(); i++) {
        MCGAL::Halfedge* hit = MCGAL::contextPool.getHalfedgeByIndex(hids[i]);
        if (!hit->isRemoved()) {
            mesh.join_face(hit);
        }
    }
}

void LocalDecoder::readBoundary() {
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

        unsigned sym = readChar(buffer, dataOffset);
        if (sym) {
            h->setBoundary();
        } else {
            h->setNotBoundary();
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

std::vector<int> LocalDecoder::decodeFacetSymbolOp(int groupId) {
    std::queue<int> gateQueue;
    gateQueue.push(seeds[groupId]->face->poolId);
    std::vector<int> fids;
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
        unsigned sym = readChar(buffer, groupOffset[groupId]);

        if (sym) {
            fids.push_back(f->poolId);
            f->setSplittable();
            f->setRemovedVertexPos(readPoint(buffer, groupOffset[groupId]));
        } else {
            f->setUnsplittable();
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
    return fids;
}

std::vector<int> LocalDecoder::decodeHalfedgeSymbolOp(int groupId) {
    std::queue<int> gateQueue;
    std::vector<int> hids;
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
        f->setProcessedFlag();
        MCGAL::Halfedge* h = hIt;
        do {
            unsigned sym = readChar(buffer, groupOffset[groupId]);

            if (sym) {
                hids.push_back(hIt->poolId);
                hIt->setAdded();
            } else {
                hIt->setOriginal();
            }
            MCGAL::Halfedge* hOpp = hIt->opposite;
            // 对方没有被处理，且该边不是边界
            if (!hOpp->face->isProcessed() && !hOpp->isBoundary()) {
                gateQueue.push(hOpp->face->poolId);
            }
            hIt = hIt->next;
        } while (hIt != h);
    }
    return hids;
}

void LocalDecoder::readMeta() {
    // 读取group数量
    int size = readInt(buffer, dataOffset);
    compressRounds.resize(size);
    groupOffset.resize(size);
    // 读取seed
    for (int i = 0; i < size; i++) {
        int st = readInt(buffer, dataOffset);
        int ed = readInt(buffer, dataOffset);
        int poolId1 = MCGAL::contextPool.vid2PoolId[st];
        int poolId2 = MCGAL::contextPool.vid2PoolId[ed];
        MCGAL::Vertex* v1 = MCGAL::contextPool.getVertexByIndex(poolId1);
        MCGAL::Vertex* v2 = MCGAL::contextPool.getVertexByIndex(poolId2);
        for (int j = 0; j < v1->halfedges.size(); j++) {
            if (v1->halfedges[j]->end_vertex == v2) {
                seeds.push_back(v1->halfedges[j]);
                break;
            }
        }
    }
    // 读取每个group的压缩轮数
    for (int i = 0; i < size; i++) {
        compressRounds[i] = readInt(buffer, dataOffset);
    }
    readBoundary();
    resetBfsState();
    // 读取每个group的offset
    for (int i = 0; i < size; i++) {
        groupOffset[i] = readInt(buffer, dataOffset);
    }
}

void LocalDecoder::resetState() {
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

void LocalDecoder::readBaseMesh() {
    unsigned i_nbVerticesBaseMesh = readInt(buffer, dataOffset);
    unsigned i_nbFacesBaseMesh = readInt(buffer, dataOffset);

    std::deque<MCGAL::Point>* p_pointDeque = new std::deque<MCGAL::Point>();
    std::deque<uint32_t*>* p_faceDeque = new std::deque<uint32_t*>();
    for (unsigned i = 0; i < i_nbVerticesBaseMesh; ++i) {
        MCGAL::Point pos = readPoint(buffer, dataOffset);
        p_pointDeque->push_back(pos);
    }
    for (unsigned i = 0; i < i_nbFacesBaseMesh; ++i) {
        int nv = readInt(buffer, dataOffset);
        uint32_t* f = new uint32_t[nv + 1];
        f[0] = nv;
        for (unsigned j = 1; j < nv + 1; ++j) {
            f[j] = readInt(buffer, dataOffset);
        }
        p_faceDeque->push_back(f);
    }
    buildFromBuffer(p_pointDeque, p_faceDeque);

    for (unsigned i = 0; i < p_faceDeque->size(); ++i) {
        delete[] p_faceDeque->at(i);
    }
    delete p_faceDeque;
    delete p_pointDeque;
}

void LocalDecoder::buildFromBuffer(std::deque<MCGAL::Point>* p_pointDeque, std::deque<uint32_t*>* p_faceDeque) {
    mesh.vertices.clear();
    std::vector<MCGAL::Vertex*> vertices;
    for (std::size_t i = 0; i < p_pointDeque->size(); ++i) {
        float x, y, z;
        MCGAL::Point p = p_pointDeque->at(i);
        MCGAL::Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(p);
        mesh.vertices.push_back(vt);
        vertices.push_back(vt);
    }
    for (int i = 0; i < p_faceDeque->size(); ++i) {
        uint32_t* ptr = p_faceDeque->at(i);
        int num_face_vertices = ptr[0];
        std::vector<MCGAL::Vertex*> vts;
        for (int j = 0; j < num_face_vertices; ++j) {
            int vertex_index = ptr[j + 1];
            vts.push_back(vertices[vertex_index]);
        }
        MCGAL::Facet* face = MCGAL::contextPool.allocateFaceFromPool(vts);
        mesh.add_face(face);
    }
    vertices.clear();
}

void LocalDecoder::loadBuffer(std::string path) {
    std::ifstream fin(path, std::ios::binary);
    int len2;
    fin.read((char*)&len2, sizeof(int));
    dataOffset = 0;
    buffer = new char[len2];
    memset(buffer, 0, len2);
    fin.read(buffer, len2);
}

void LocalDecoder::resetBfsState() {
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