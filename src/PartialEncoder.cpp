#include "./include/PartialEncoder.h"
#include <iostream>

MCGAL::Halfedge* PartialEncoder::next_boundary(int ogroupId, MCGAL::Halfedge* boundary) {
    MCGAL::Halfedge* nxt = boundary->next;
    if (nxt->isBoundary()) {
        auto ddd = origin2dup[dup2origin[nxt->end_vertex->id]];
        assert(origin2dup[dup2origin[nxt->end_vertex->id]].count(ogroupId));
        return nxt;
    }
    nxt = boundary->next->opposite->next;
    if (nxt->isBoundary()) {
        auto ddd = origin2dup[dup2origin[nxt->end_vertex->id]];
        assert(origin2dup[dup2origin[nxt->end_vertex->id]].count(ogroupId));
        return nxt;
    }
    for (MCGAL::Halfedge* hit : boundary->end_vertex->halfedges) {
        if (hit->opposite != boundary && hit->isBoundary()) {
            auto ddd = origin2dup[dup2origin[nxt->end_vertex->id]];
            assert(origin2dup[dup2origin[hit->end_vertex->id]].count(ogroupId));
            return hit;
        }
    }
    return nullptr;
}

PartialEncoder::PartialEncoder(std::string path) {
    mesh.loadOFF(path);
    splitter.loadMesh(&mesh);
    splitter.split();
    subMeshes = std::move(splitter.exportSubMeshes());
    seeds = splitter.exportSeeds();
    graph = splitter.exportGraph();
    origin2dup = splitter.exportOrigin2Dup();
    dup2origin = splitter.exportDup2Origin();
}

void PartialEncoder::encode(int round) {
    int cur_round = 0;
    while (cur_round < round) {
        for (int i = 0; i < subMeshes.size(); i++) {
            encodeInsideOp(i);
            resetState(i);
        }
        for (int i = 0; i < subMeshes.size(); i++) {
            encodeBoundaryOp(i);
        }
        for (int i = 0; i < subMeshes.size(); i++) {
            std::string path = "./submesh1/round" + std::to_string(cur_round) + "_group" + std::to_string(i) + ".off";
            subMeshes[i].dumpto(path);
            resetState(i);
            std::unordered_map<int, Node>& nodes = graph.getNode(i);
            for (auto& [_, node] : nodes) {
                node.setUnvisiable();
            }
        }
        cur_round++;
    }
}

void PartialEncoder::encode(int groupId, int round) {
    int cur_round = 0;
    std::string path = "./submesh2/round" + std::to_string(-1) + "_group" + std::to_string(groupId) + ".off";
    subMeshes[groupId].dumpto(path);
    while (cur_round < round) {
        encodeInsideOp(groupId);
        encodeBoundaryOp(groupId);
        std::string path = "./submesh2/round" + std::to_string(cur_round) + "_group" + std::to_string(groupId) + ".off";
        subMeshes[groupId].dumpto(path);
        dumpSubMeshNeighbour(groupId, cur_round);
        cur_round++;
        resetState(groupId);
    }
}

void PartialEncoder::dumpSubMeshNeighbour(int groupId, int round) {
    std::unordered_map<int, Node> nodes = graph.getNode(groupId);
    for (auto& [neighbourId, node] : nodes) {
        int id = neighbourId;
        std::string path = "./submesh2/round" + std::to_string(round) + "_group" + std::to_string(id) + ".off";
        subMeshes[id].dumpto(path);
    }
}

void PartialEncoder::encodeInsideOp(int groupId) {
    MCGAL::Mesh& submesh = subMeshes[groupId];
    std::queue<int> gateQueue;
    gateQueue.push(seeds[groupId]->poolId);
    int removedCount = 0;
    std::set<int> boundaryIds;
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
            if (isRemovable(groupId, hh->end_vertex) && !hh->isBoundary() && !hh->opposite->isBoundary()) {
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
                if (hOpp != nullptr && !hOpp->isBoundary() && !h->isBoundary() && hOpp->face->groupId == groupId) {
                    gateQueue.push(hOpp->poolId);
                }
            } while ((hh = hh->next) != h);
        } else {
            removedCount++;
            vertexCut(submesh, boundaryIds, gateQueue, unconqueredVertexHE);
        }
    }
    // if (removedCount != 0) {
    //     resetBfsState();
    //     compressRounds[groupId]++;
    //     encodeFacetSymbolOp(groupId);
    //     resetBfsState();
    //     encodeHalfedgeSymbolOp(groupId);
    //                                                 resetBfsState();
    //     // encodeLocalBoundary(groupId, boundarys);
    // }
}

void PartialEncoder::encodeBoundaryOp(int groupId) {
    // halfedge collapse
    // for (int id : boundaryIds) {
    //     MCGAL::Halfedge* hit = MCGAL::contextPool.getHalfedgeByIndex(id);
    //     if (isBoundaryRemovable(hit)) {
    //         MCGAL::Vertex* newv = subMeshes[groupId].halfedge_collapse(hit);
    //         for (MCGAL::Halfedge* h : newv->halfedges) {
    //             if (h->isBoundary()) {
    //                 h->setCantCollapse();
    //                 h->opposite->setCantCollapse();
    //             }
    //             if (h->face->groupId != h->opposite->face->groupId) {
    //                 if (!h->isBoundary()) {
    //                     h->setBoundary();
    //                     h->setCantCollapse();
    //                     h->opposite->setCantCollapse();
    //                     h->opposite->setBoundary();
    //                 } else {
    //                 }
    //             }
    //         }
    //     }
    // }

    // vertex removal
    std::unordered_map<int, Node>& nodes = graph.getNode(groupId);
    for (auto& [neighbourId, node] : nodes) {
        if (node.isVisiable()) {
            continue;
        }
        node.setVisiable();
        std::unordered_map<int, int>& groupId2dup = origin2dup[node.st];
        int dupId = groupId2dup[groupId];
        int poolId = MCGAL::contextPool.vid2PoolId[dupId];
        MCGAL::Vertex* vt = MCGAL::contextPool.getVertexByIndex(poolId);
        MCGAL::Halfedge* boundary = nullptr;
        for (MCGAL::Halfedge* hit : vt->halfedges) {
            if (hit->isBoundary()) {
                if (dup2origin[hit->end_vertex->id] == node.ed) {
                    auto ddd = origin2dup[node.ed];
                    boundary = hit;
                    break;
                }
            }
        }

        std::unordered_map<int, Node>& neighbour_nodes = graph.getNode(neighbourId);
        for (auto& [key, nd] : neighbour_nodes) {
            if (key == groupId) {
                nd.setVisiable();
                break;
            }
        }

        // 规定了走向后，也要规定什么时候结束
        // int edPoolId = -1;
        // std::unordered_set<Node, Hasher> neighboures = graph.getNode(node.neighbour);
        // for (Node neighbour : neighboures) {
        //     if (neighbour.neighbour == groupId) {
        //         std::unordered_map<int, int>& groupId2dup = origin2dup[neighbour.st];
        //         int dupId = groupId2dup[groupId];
        //         edPoolId = MCGAL::contextPool.vid2PoolId[dupId];
        //         break;
        //     }
        // }
        // MCGAL::Halfedge* ed = boundary;
        int stopId = origin2dup[node.stop][groupId];
        // NEW_boundary 可能有·问题
        int cnt = 0;
        do {
            if (boundaryRemovableInVertexRemoval(groupId, neighbourId, boundary)) {
                if (boundary->face->facet_degree() == 3) {
                    boundary->face->setRemoved();
                    boundary->end_vertex->setRemoved();
                    boundary->next->next->opposite->setRemovedVertex(boundary->end_vertex);
                    boundary->next->next->opposite->setBoundary();
                    for (MCGAL::Halfedge* hit : boundary->face->halfedges) {
                        hit->setRemoved();
                        hit->vertex->eraseHalfedgeByPointer(hit);
                    }
                    MCGAL::Halfedge* new_boundary = boundary->next->next->opposite;
                    // 这两个点之间构造一个边
                    MCGAL::Vertex* v1 = boundary->vertex;
                    MCGAL::Vertex* v2 = boundary->next->end_vertex;
                    // 获得对面的groupId
                    int removedVertexVid = origin2dup[dup2origin[boundary->end_vertex->id]][neighbourId];
                    int oppoVid1 = origin2dup[dup2origin[v1->id]][neighbourId];
                    int oppoVid2 = origin2dup[dup2origin[v2->id]][neighbourId];
                    MCGAL::Vertex* removedVertex = MCGAL::contextPool.getVertexByVid(removedVertexVid);
                    MCGAL::Vertex* ov1 = MCGAL::contextPool.getVertexByVid(oppoVid1);
                    MCGAL::Vertex* ov2 = MCGAL::contextPool.getVertexByVid(oppoVid2);
                    /**
                     * 指向关系应该是 removedVertex-> v2 -> v1
                     */
                    // 找到问题，不符合流行结构，需要进行检测
                    MCGAL::Facet* onew_face = MCGAL::contextPool.allocateFaceFromPool({removedVertex, ov2, ov1});
                    onew_face->setGroupId(neighbourId);
                    MCGAL::Halfedge* startH;
                    bool fg1 = false, fg2 = false, fg3 = false;
                    for (MCGAL::Halfedge* hit : ov1->halfedges) {
                        if (hit->end_vertex == removedVertex) {
                            fg1 = true;
                            startH = hit;
                        }
                    }
                    for (MCGAL::Halfedge* hit : ov2->halfedges) {
                        if (hit->end_vertex == ov1) {
                            fg2 = true;
                            hit->setBoundary();
                            hit->opposite = nullptr;
                            break;
                        }
                    }
                    for (MCGAL::Halfedge* hit : ov2->halfedges) {
                        if (hit->end_vertex == removedVertex) {
                            fg3 = true;
                            hit->setNotBoundary();
                            break;
                        }
                    }
                    for (MCGAL::Halfedge* hit : removedVertex->halfedges) {
                        if (hit->end_vertex == ov1) {
                            hit->setNotBoundary();
                            break;
                        }
                    }
                    assert(fg1 && fg2 && fg3);
                    subMeshes[neighbourId].add_face(onew_face);
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
                            MCGAL::Halfedge* hCorner = subMeshes[neighbourId].split_facet(h, hSplit);
                            hCorner->setAdded();
                        }
                        h->end_vertex->setConquered();
                        removed++;
                    } while ((h = h->opposite->next) != end);

                    MCGAL::Halfedge* newH = subMeshes[neighbourId].erase_center_vertex(startH);
                    newH->face->setGroupId(neighbourId);
                    subMeshes[neighbourId].add_face(newH->face);
                    boundary = new_boundary;
                } else if (boundary->face->facet_degree() > 3) {
                    // 这两个点之间构造一个边
                    MCGAL::Vertex* v1 = boundary->vertex;
                    MCGAL::Vertex* v2 = boundary->next->end_vertex;
                    int oriRemovedVid = boundary->end_vertex->id;

                    // 从大的多边形中移除小的多边形
                    boundary->end_vertex->setRemoved();
                    boundary->setRemovedVertex(boundary->end_vertex);
                    boundary->end_vertex = boundary->next->end_vertex;

                    // 减除一个三角形
                    boundary->next = boundary->next->next;
                    boundary->face->reset(boundary);

                    // 获得对面的groupId
                    int removedVertexVid = origin2dup[dup2origin[oriRemovedVid]][neighbourId];
                    int oppoVid1 = origin2dup[dup2origin[v1->id]][neighbourId];
                    int oppoVid2 = origin2dup[dup2origin[v2->id]][neighbourId];
                    MCGAL::Vertex* removedVertex = MCGAL::contextPool.getVertexByVid(removedVertexVid);
                    MCGAL::Vertex* ov1 = MCGAL::contextPool.getVertexByVid(oppoVid1);
                    MCGAL::Vertex* ov2 = MCGAL::contextPool.getVertexByVid(oppoVid2);
                    /**
                     * 指向关系应该是 removedVretex-> v2 -> v1
                     */
                    MCGAL::Facet* onew_face = MCGAL::contextPool.allocateFaceFromPool({removedVertex, ov2, ov1});
                    MCGAL::Halfedge* startH;
                    for (MCGAL::Halfedge* hit : ov1->halfedges) {
                        if (hit->end_vertex == removedVertex) {
                            startH = hit;
                        }
                    }
                    bool fg1 = false, fg2 = false, fg3 = false;
                    for (MCGAL::Halfedge* hit : ov2->halfedges) {
                        if (hit->end_vertex == ov1) {
                            fg1 = true;
                            hit->setBoundary();
                            hit->opposite = nullptr;
                            break;
                        }
                    }
                    for (MCGAL::Halfedge* hit : ov2->halfedges) {
                        if (hit->end_vertex == removedVertex) {
                            fg2 = true;
                            hit->setNotBoundary();
                            break;
                        }
                    }
                    for (MCGAL::Halfedge* hit : removedVertex->halfedges) {
                        if (hit->end_vertex == ov1) {
                            fg3 = true;
                            hit->setNotBoundary();
                            break;
                        }
                    }
                    assert(fg1 && fg2 && fg3);
                    subMeshes[neighbourId].add_face(onew_face);
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
                            MCGAL::Halfedge* hCorner = subMeshes[neighbourId].split_facet(h, hSplit);
                            hCorner->setAdded();
                        }
                        h->end_vertex->setConquered();
                        removed++;
                    } while ((h = h->opposite->next) != end);

                    MCGAL::Halfedge* newH = subMeshes[neighbourId].erase_center_vertex(startH);
                    newH->face->setGroupId(neighbourId);
                    subMeshes[neighbourId].add_face(newH->face);
                    // boundary = new_boundary;
                }
            }
            if (boundary->end_vertex->id == stopId) {
                break;
            }
            boundary = next_boundary(neighbourId, boundary);
            cnt++;
        } while (boundary->end_vertex->id != stopId);
        std::cout << "boundary number: " << cnt << std::endl;
    }
}

MCGAL::Halfedge* PartialEncoder::vertexCut(MCGAL::Mesh& submesh, std::set<int>& boundaryIds, std::queue<int>& gateQueue, MCGAL::Halfedge* startH) {
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
            MCGAL::Halfedge* hCorner = submesh.split_facet(h, hSplit);
            hCorner->setAdded();
        }
        h->end_vertex->setConquered();
        removed++;
    } while ((h = h->opposite->next) != end);
    MCGAL::Point vPos = startH->end_vertex->point();
    MCGAL::Halfedge* hNewFace = submesh.erase_center_vertex(startH);
    MCGAL::Facet* added_face = hNewFace->face;
    added_face->setSplittable();
    added_face->setRemovedVertexPos(vPos);
    h = hNewFace;
    do {
        MCGAL::Halfedge* hOpp = h->opposite;
        if (h->isBoundary()) {
            boundaryIds.insert(h->poolId);
        }
        if (hOpp == nullptr) {
            continue;
        }
        if (!hOpp->face->isConquered() && !hOpp->isBoundary() && !h->isBoundary()) {
            gateQueue.push(hOpp->poolId);
        }
    } while ((h = h->next) != hNewFace);
    return hNewFace;
}

void PartialEncoder::resetState(int groupId) {
    MCGAL::Mesh& subMesh = subMeshes[groupId];
    // 假设mesh.vertices和mesh.faces都是std::vector的实例
    auto vertices_end = std::remove_if(subMesh.vertices.begin(), subMesh.vertices.end(), [](MCGAL::Vertex* vertex) { return vertex->isRemoved(); });
    subMesh.vertices.erase(vertices_end, subMesh.vertices.end());

    auto faces_end = std::remove_if(subMesh.faces.begin(), subMesh.faces.end(), [](MCGAL::Facet* face) { return face->isRemoved(); });
    subMesh.faces.erase(faces_end, subMesh.faces.end());

    // 对保留的顶点和面执行resetState()
    std::for_each(subMesh.vertices.begin(), subMesh.vertices.end(), [](MCGAL::Vertex* vertex) { vertex->resetState(); });

    std::for_each(subMesh.faces.begin(), subMesh.faces.end(), [](MCGAL::Facet* face) {
        face->resetState();
        // 对于每个半边，检查并设置边界状态，然后重置状态
        for (MCGAL::Halfedge* hit : face->halfedges) {
            if (hit->opposite == nullptr || hit->face->groupId != hit->opposite->face->groupId) {
                hit->setBoundary();
            }
            hit->resetState();
        }
    });
}

bool PartialEncoder::boundaryRemovableInVertexRemoval(int inner, int outer, MCGAL::Halfedge* hit) {
    assert(hit->isBoundary());
    std::unordered_map<int, int> map1 = origin2dup[dup2origin[hit->vertex->id]];
    int t = dup2origin[hit->end_vertex->id];
    std::unordered_map<int, int> map2 = origin2dup[dup2origin[hit->end_vertex->id]];
    if (map1.size() >= 3 || map2.size() >= 3) {
        return false;
    }
    if (hit->isBoundary() && hit->next->isBoundary()) {
        // return true;
        MCGAL::Vertex* vit = hit->end_vertex;
        int t = dup2origin[hit->end_vertex->id];
        auto ddd = origin2dup[dup2origin[hit->end_vertex->id]];
        assert(origin2dup[dup2origin[hit->end_vertex->id]].count(outer));
        int odupId = origin2dup[dup2origin[hit->end_vertex->id]][outer];
        MCGAL::Vertex* ovit = MCGAL::contextPool.getVertexByVid(odupId);
        if (vit->halfedges.size() + ovit->halfedges.size() < 8) {
            std::vector<MCGAL::Point> vh_oneRing;
            std::vector<MCGAL::Halfedge*> inner_oneRing;
            std::vector<MCGAL::Halfedge*> outer_oneRing;
            inner_oneRing.reserve(vit->halfedges.size());
            outer_oneRing.reserve(ovit->halfedges.size());

            /** 简单粗暴一点 */
            int vid1 = hit->vertex->id;
            int vid2 = hit->end_vertex->id;
            int vid3 = hit->next->end_vertex->id;
            int ovid1 = origin2dup[dup2origin[vid1]][outer];
            int ovid2 = origin2dup[dup2origin[vid2]][outer];
            int ovid3 = origin2dup[dup2origin[vid3]][outer];

            MCGAL::Vertex* ovit1 = MCGAL::contextPool.getVertexByVid(ovid1);
            MCGAL::Vertex* ovit2 = MCGAL::contextPool.getVertexByVid(ovid2);
            MCGAL::Vertex* ovit3 = MCGAL::contextPool.getVertexByVid(ovid3);

            MCGAL::Halfedge* ohit1 = nullptr;
            MCGAL::Halfedge* ohit2 = nullptr;
            for (MCGAL::Halfedge* ohit : ovit2->halfedges) {
                if (ohit->end_vertex == ovit1) {
                    ohit1 = ohit;
                }
            }
            for (MCGAL::Halfedge* ohit : ovit3->halfedges) {
                if (ohit->end_vertex == ovit2) {
                    ohit2 = ohit;
                }
            }
            if (ohit1->face->poolId == ohit2->face->poolId) {
                return false;
            }

            for (MCGAL::Halfedge* hit : vit->halfedges) {
                vh_oneRing.push_back(hit->end_vertex->point());
                inner_oneRing.push_back(hit);
            }
            for (MCGAL::Halfedge* hit : vit->halfedges) {
                vh_oneRing.push_back(hit->end_vertex->point());
                outer_oneRing.push_back(hit);
            }
            return !willViolateManifoldInDup(inner, outer, inner_oneRing, outer_oneRing) && arePointsCoplanar(vh_oneRing);
        }
    }
    return false;
}

bool PartialEncoder::isBoundaryRemovable(MCGAL::Halfedge* h) {
    bool res = false;
    for (int i = 0; i < seeds.size(); i++) {
        if (seeds[i]->poolId == h->poolId) {
            return false;
        }
    }
    std::set<int> poolIds;
    for (MCGAL::Halfedge* hit : h->face->halfedges) {
        if (poolIds.count(hit->opposite->face->poolId)) {
            return false;
        }
        poolIds.insert(hit->opposite->face->poolId);
    }
    poolIds.clear();
    for (MCGAL::Halfedge* hit : h->opposite->face->halfedges) {
        if (poolIds.count(hit->opposite->face->poolId)) {
            return false;
        }
        poolIds.insert(hit->opposite->face->poolId);
    }
    std::vector<MCGAL::Halfedge*> hs;
    poolIds.clear();
    for (MCGAL::Halfedge* hit : h->vertex->halfedges) {
        if (poolIds.count(hit->end_vertex->poolId)) {
            continue;
        }
        hs.push_back(hit);
    }
    for (MCGAL::Halfedge* hit : h->end_vertex->halfedges) {
        if (poolIds.count(hit->end_vertex->poolId)) {
            continue;
        }
        hs.push_back(hit);
    }

    return h->canCollapse() && !h->isRemoved() && h->face->facet_degree() == 3 && h->opposite->face->facet_degree() == 3 && !willViolateManifold(hs);
}

bool PartialEncoder::isRemovable(int groupId, MCGAL::Vertex* v) {
    for (int i = 0; i < seeds.size(); i++) {
        if (v->poolId == seeds[i]->vertex->poolId || v->poolId == seeds[i]->end_vertex->poolId) {
            return false;
        }
    }

    // std::unordered_set<Node>& nodes = graph.getNode(groupId);
    // for (const Node& node : nodes) {
    //     assert(origin2dup[node.st].count(groupId));
    //     assert(origin2dup[node.ed].count(groupId));
    //     if (origin2dup[node.st][groupId] == v->id) {
    //         return false;
    //     }
    //     if (origin2dup[node.ed][groupId] == v->id) {
    //         return false;
    //     }
    // }

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
        bool removable = !willViolateManifold(heh_oneRing) && arePointsCoplanar(vh_oneRing);
        // if (removable) {
        //     return checkCompetition(v);
        // }
        return removable;
    }
    return false;
}

bool PartialEncoder::arePointsCoplanar(std::vector<MCGAL::Point>& points) {
    if (points.size() < 4)
        return true;  // 三个点总是共面的

    // 选择前三个点，计算两个向量
    MCGAL::Vector v1 = points[1] - points[0];
    MCGAL::Vector v2 = points[2] - points[0];

    // 计算法向量
    MCGAL::Vector n = v1.cross(v2);

    // 检查剩余点是否在平面上
    for (size_t i = 3; i < points.size(); ++i) {
        MCGAL::Vector vi = points[i] - points[0];
        if (std::abs(n.dot(vi)) > 1e-8) {  // 如果点不在平面上
            return false;
        }
    }
    return true;
}

bool PartialEncoder::checkCompetition(MCGAL::Vertex* v) const {
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

bool PartialEncoder::willViolateManifold(const std::vector<MCGAL::Halfedge*>& polygon) const {
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

bool PartialEncoder::willViolateManifoldInDup(int inner, int outer, const std::vector<MCGAL::Halfedge*>& inner_oneRing, const std::vector<MCGAL::Halfedge*>& outer_oneRing) {
    unsigned i_degree = inner_oneRing.size();

    for (int i = 0; i < inner_oneRing.size(); i++) {
        MCGAL::Halfedge* it = inner_oneRing[i];
        if (it->face->facet_degree() == 3) {
            continue;
        }
        for (int j = 0; j < inner_oneRing.size(); j++) {
            MCGAL::Halfedge* jt = inner_oneRing[j];
            if (i == j)
                continue;
            if (jt->opposite == nullptr) {
                MCGAL::Vertex* vit1 = it->vertex;
                MCGAL::Vertex* vit2 = it->end_vertex;
                int vid1 = origin2dup[dup2origin[it->vertex->id]][outer];
                int vid2 = origin2dup[dup2origin[it->end_vertex->id]][outer];
                MCGAL::Vertex* v1 = MCGAL::contextPool.getVertexByVid(vid1);
                MCGAL::Vertex* v2 = MCGAL::contextPool.getVertexByVid(vid2);
                MCGAL::Halfedge* ohit1 = nullptr;
                MCGAL::Halfedge* ohit2 = nullptr;
                for (MCGAL::Halfedge* hit : v2->halfedges) {
                    if (hit->end_vertex == v1) {
                        ohit1 = hit;
                    }
                }
                for (MCGAL::Halfedge* outer_hit : outer_oneRing) {
                    if (outer_hit->face->poolId == ohit2->face->poolId) {
                        ohit2 = outer_hit;
                    }
                }

                for (int k = 0; k < v2->halfedges.size(); k++) {
                    if (v2->halfedges[k]->end_vertex == ohit2->end_vertex) {
                        return true;
                    }
                }

            } else if (it->face == jt->opposite->face) {
                for (int k = 0; k < it->end_vertex->halfedges.size(); k++) {
                    if (it->end_vertex->halfedges[k]->end_vertex == jt->end_vertex) {
                        return true;
                    }
                }
            }
        }
    }

    for (int i = 0; i < outer_oneRing.size(); i++) {
        MCGAL::Halfedge* it = outer_oneRing[i];
        if (it->face->facet_degree() == 3) {
            continue;
        }
        for (int j = 0; j < outer_oneRing.size(); j++) {
            MCGAL::Halfedge* jt = outer_oneRing[j];
            if (i == j)
                continue;
            if (jt->opposite == nullptr) {
                MCGAL::Vertex* vit1 = it->vertex;
                MCGAL::Vertex* vit2 = it->end_vertex;
                int vid1 = origin2dup[dup2origin[it->vertex->id]][inner];
                int vid2 = origin2dup[dup2origin[it->end_vertex->id]][inner];
                MCGAL::Vertex* v1 = MCGAL::contextPool.getVertexByVid(vid1);
                MCGAL::Vertex* v2 = MCGAL::contextPool.getVertexByVid(vid2);
                MCGAL::Halfedge* ohit1 = nullptr;
                MCGAL::Halfedge* ohit2 = nullptr;
                for (MCGAL::Halfedge* hit : v2->halfedges) {
                    if (hit->end_vertex == v1) {
                        ohit1 = hit;
                    }
                }
                for (MCGAL::Halfedge* outer_hit : outer_oneRing) {
                    if (outer_hit->face->poolId == ohit2->face->poolId) {
                        ohit2 = outer_hit;
                    }
                }

                for (int k = 0; k < v2->halfedges.size(); k++) {
                    if (v2->halfedges[k]->end_vertex == ohit2->end_vertex) {
                        return true;
                    }
                }

            } else if (it->face == jt->opposite->face) {
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