#include "include/Facet.h"
#include "include/Global.h"
#include "include/Halfedge.h"
#include "include/Vertex.h"
#include "stdio.h"

namespace MCGAL {
void Facet::remove(Halfedge* rh) {
    // halfedges.erase(rh);
    for (auto hit = halfedges.begin(); hit != halfedges.end();) {
        if ((*hit) == rh) {
            halfedges.erase(hit);
        } else {
            hit++;
        }
    }

    for (Halfedge* h : halfedges) {
        if (h->next == rh) {
            h->next = nullptr;
        }
    }
    delete rh;
}

Point Facet::getRemovedVertexPos() const {
    return removedVertexPos;
};

void Facet::setRemovedVertexPos(Point p) {
    removedVertexPos = p;
};

Facet::Facet(const Facet& face) {
    // this->vertices = face.vertices;
    // this->halfedges = face.halfedges;
    this->flag = face.flag;
    this->processedFlag = face.processedFlag;
}

Facet::~Facet() {
    halfedges.clear();
    vertices.clear();
}

Facet::Facet(Halfedge* hit) {
    // vertices.reserve(SMALL_BUCKET_SIZE);
    // halfedges.reserve(SMALL_BUCKET_SIZE);
    Halfedge* st(hit);
    Halfedge* ed(hit);
    std::vector<Halfedge*> edges;
    do {
        edges.push_back(st);
        st = st->next;
    } while (st != ed);
    this->reset(edges);
}

Facet* Facet::clone() {
    return new Facet(*this);
}

// Facet::Facet(std::vector<Vertex*>& vs) {
//     // vertices.reserve(SMALL_BUCKET_SIZE);
//     // halfedges.reserve(SMALL_BUCKET_SIZE);
//     Halfedge* prev = nullptr;
//     Halfedge* head = nullptr;
//     for (int i = 0; i < vs.size(); i++) {
//         vertices.push_back(vs[i]);
//         Vertex* nextv = vs[(i + 1) % vs.size()];
//         Halfedge* hf = new Halfedge(vs[i], nextv);
//         halfedges.push_back(hf);
//         // vs[i]->halfedges.insert(hf);
//         hf->face = this;
//         if (prev != nullptr) {
//             prev->next = hf;
//         } else {
//             head = hf;
//         }
//         if (i == vs.size() - 1) {
//             hf->next = head;
//         }
//         prev = hf;
//     }
// }

Facet::Facet(std::vector<Vertex*>& vs) {
    // vertices.reserve(SMALL_BUCKET_SIZE);
    // halfedges.reserve(SMALL_BUCKET_SIZE);
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < vs.size(); i++) {
        vertices.push_back(vs[i]);
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(vs[i], nextv);
        halfedges.push_back(hf);
        // vs[i]->halfedges.insert(hf);
        hf->face = this;
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
}

// Facet::Facet(std::vector<Vertex*>& vs, Mesh* mesh) {
//     // vertices.reserve(SMALL_BUCKET_SIZE);
//     // halfedges.reserve(SMALL_BUCKET_SIZE);
//     Halfedge* prev = nullptr;
//     Halfedge* head = nullptr;
//     for (int i = 0; i < vs.size(); i++) {
//         vertices.push_back(vs[i]);
//         Vertex* nextv = vs[(i + 1) % vs.size()];
//         Halfedge* hf = std::move(mesh->allocateHalfedgeFromPool(vs[i], nextv));
//         halfedges.push_back(hf);
//         // vs[i]->halfedges.insert(hf);
//         hf->face = this;
//         if (prev != NULL) {
//             prev->next = hf;
//         } else {
//             head = hf;
//         }
//         if (i == vs.size() - 1) {
//             hf->next = head;
//         }
//         prev = hf;
//     }
// }

void Facet::reset(std::vector<Vertex*>& vs) {
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < vs.size(); i++) {
        vertices.push_back(vs[i]);
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(vs[i], nextv);
        halfedges.push_back(hf);
        // vs[i]->halfedges.insert(hf);
        hf->face = this;
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
}

// void Facet::reset(std::vector<Vertex*>& vs, Mesh* mesh) {
//     Halfedge* prev = nullptr;
//     Halfedge* head = nullptr;
//     for (int i = 0; i < vs.size(); i++) {
//         vertices.push_back(vs[i]);
//         Vertex* nextv = vs[(i + 1) % vs.size()];
//         Halfedge* hf = mesh->allocateHalfedgeFromPool(vs[i], nextv);
//         halfedges.push_back(hf);
//         // vs[i]->halfedges.insert(hf);
//         hf->face = this;
//         if (prev != NULL) {
//             prev->next = hf;
//         } else {
//             head = hf;
//         }
//         if (i == vs.size() - 1) {
//             hf->next = head;
//         }
//         prev = hf;
//     }
// }

void Facet::reset(Halfedge* h) {
    Halfedge* st = h;
    Halfedge* ed = h;
    std::vector<Halfedge*> edges;
    do {
        edges.push_back(st);
        st = st->next;
    } while (st != ed);
    reset(edges);
}

// void Facet::reset(Halfedge* h) {
//     Halfedge* st = h;
//     Halfedge* ed = h;
//     std::vector<Halfedge*> edges;
//     this->halfedges.clear();
//     this->vertices.clear();
//     do {
//         // edges.push_back(st);
//         this->halfedges.push_back(st);
//         this->vertices.push_back(st->vertex);
//         st->face = this;
//         st = st->next;
//     } while (st != ed);
//     // reset(edges);
// }

void Facet::reset(std::vector<Halfedge*>& hs) {
    this->halfedges = hs;
    // this->halfedges.shrink_to_fit();
    // this->halfedges.reserve(10);
    this->vertices.clear();
    this->vertices.shrink_to_fit();
    this->vertices.reserve(10);
    for (int i = 0; i < hs.size(); i++) {
        // hs[i]->next = hs[(i + 1) % hs.size()];
        // hs[i]->face = this;
        // this->halfedges.push_back(hs[i]);
        this->vertices.push_back(hs[i]->vertex);
        hs[i]->face = this;
    }
}

void Facet::print() {
    printf("totally %ld vertices:\n", vertices.size());
    int idx = 0;
    for (Vertex* v : vertices) {
        printf("%d:\t", idx++);
        v->print();
    }
}

void Facet::print_off() {
    printf("OFF\n%ld 1 0\n", vertices.size());
    for (Vertex* v : vertices) {
        v->print();
    }
    printf("%ld\t", vertices.size());
    for (int i = 0; i < vertices.size(); i++) {
        printf("%d ", i);
    }
    printf("\n");
}

bool Facet::equal(const Facet& rhs) const {
    if (vertices.size() != rhs.vertices.size()) {
        return false;
    }
    for (Vertex* vertix : vertices) {
        for (Vertex* vt : rhs.vertices) {
            if (vt == vertix) {
                return true;
            }
        }
    }

    return false;
}
bool Facet::operator==(const Facet& rhs) const {
    return this->equal(rhs);
}

int Facet::facet_degree() {
    return vertices.size();
}

}  // namespace MCGAL
