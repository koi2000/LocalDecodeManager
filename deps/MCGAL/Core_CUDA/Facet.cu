#include "include/Facet.cuh"
#include "include/global.cuh"
namespace MCGAL {

// void Facet::remove(Halfedge* rh) {
//     // halfedges.erase(rh);
//     for (auto hit = halfedges.begin(); hit != halfedges.end();) {
//         if ((*hit) == rh) {
//             halfedges.erase(hit);
//         } else {
//             hit++;
//         }
//     }

//     for (Halfedge* h : halfedges) {
//         if (h->next == rh) {
//             h->next = NULL;
//         }
//     }
//     delete rh;
// }

Facet::Facet(const Facet& face) {
    // this->vertices = face.vertices;
    // this->halfedges = face.halfedges;
    this->flag = face.flag;
    this->processedFlag = face.processedFlag;
}

Facet::~Facet() {}

Facet::Facet(Halfedge* hit) {
    // vertices.reserve(SMALL_BUCKET_SIZE);
    // halfedges.reserve(SMALL_BUCKET_SIZE);
    Halfedge* st(hit);
    Halfedge* ed(hit);
    std::vector<Halfedge*> edges;
    do {
        edges.push_back(st);
        st = st->next();
    } while (st != ed);
    this->reset(edges);
}

void Facet::addHalfedge(Halfedge* halfedge) {
    // halfedge->setFacet(this);
    halfedges[halfedge_size++] = halfedge->poolId;
};

void Facet::addHalfedge(int halfedge) {
    // MCGAL::contextPool.getHalfedgeByIndex(halfedge)->setFacet(this);
    halfedges[halfedge_size++] = halfedge;
};

void Facet::addVertex(Vertex* vertex) {
    vertices[vertex_size++] = vertex->poolId;
};

void Facet::addVertex(int vertex) {
    vertices[vertex_size++] = vertex;
};

Vertex* Facet::getVertexByIndex(int index) {
    assert(index < vertex_size);
    return contextPool.getVertexByIndex(vertices[index]);
}

Halfedge* Facet::getHalfedgeByIndex(int index) {
    assert(index < halfedge_size);
    return contextPool.getHalfedgeByIndex(halfedges[index]);
}

Facet* Facet::clone() {
    return new Facet(*this);
}

Facet::Facet(std::vector<Vertex*>& vs) {
    // vertices.reserve(SMALL_BUCKET_SIZE);
    // halfedges.reserve(SMALL_BUCKET_SIZE);
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < vs.size(); i++) {
        // vertices.push_back(vs[i]);
        addVertex(vs[i]);
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = contextPool.allocateHalfedgeFromPool(vs[i], nextv);
        // halfedges.push_back(hf);
        addHalfedge(hf);
        // vs[i]->halfedges.insert(hf);
        // hf->face = this;
        hf->setFacet(this);
        if (prev != NULL) {
            // prev->next = hf;
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == vs.size() - 1) {
            // hf->next = head;
            hf->setNext(head);
        }
        prev = hf;
    }
}

void Facet::reset(std::vector<Vertex*>& vs) {
    halfedge_size = 0;
    vertex_size = 0;
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < vs.size(); i++) {
        // vertices.push_back(vs[i]);
        addVertex(vs[i]);
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = contextPool.allocateHalfedgeFromPool(vs[i], nextv);
        // halfedges.push_back(hf);
        addHalfedge(hf);
        // vs[i]->halfedges.insert(hf);
        // hf->face = this;
        hf->setFacet(this);
        if (prev != NULL) {
            // prev->next = hf;
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == vs.size() - 1) {
            // hf->next = head;
            hf->setNext(head);
        }
        prev = hf;
    }
}

void Facet::reset(Halfedge* h) {
    Halfedge* st = h;
    Halfedge* ed = h;
    std::vector<Halfedge*> edges;
    do {
        edges.push_back(st);
        st = st->next();
    } while (st != ed);
    reset(edges);
}

// __device__ void Facet::resetOnCuda(Vertex* vertices, Halfedge* halfedges, Halfedge* h) {
//     halfedge_size = 0;
//     vertex_size = 0;
//     Halfedge* st = h;
//     addHalfedgeOnCuda(st);
//     addVertexOnCuda(st->dvertex(vertices));
//     st->setFacetOnCuda(this);
//     st = st->dnext(halfedges);
//     addHalfedgeOnCuda(st);
//     addVertexOnCuda(st->dvertex(vertices));
//     st->setFacetOnCuda(this);
//     st = st->dnext(halfedges);
//     addHalfedgeOnCuda(st);
//     addVertexOnCuda(st->dvertex(vertices));
//     st->setFacetOnCuda(this);
//     st = st->dnext(halfedges);
// }

__device__ void Facet::resetOnCuda(Vertex* vertices, Halfedge* halfedges, Halfedge* h) {
    int tid = blockIdx.x * (blockDim.x * blockDim.y) + blockDim.x * threadIdx.y + threadIdx.x;
    halfedge_size = 0;
    vertex_size = 0;
    Halfedge* st = h;
    Halfedge* ed = h;
    int idx = 0;
    do {
        idx++;
        if (idx >= 120) {
            // printf("error %d ", this->poolId);
            break;
        }
        addHalfedgeOnCuda(st);
        // this->vertices.push_back(hs[i]->vertex);
        addVertexOnCuda(st->dvertex(vertices));
        st->setFacetOnCuda(this);
        st = st->dnext(halfedges);
    } while (st != ed);
}

void Facet::reset(std::vector<Halfedge*>& hs) {
    halfedge_size = 0;
    vertex_size = 0;
    for (int i = 0; i < hs.size(); i++) {
        // this->halfedges.push_back(hs[i]);
        addHalfedge(hs[i]);
        // this->vertices.push_back(hs[i]->vertex);
        addVertex(hs[i]->vertex());
        hs[i]->setFacet(this);
        // hs[i]->face = this;
    }
}

void Facet::print() {
    printf("totally %d vertices:\n", vertex_size);
    int idx = 0;
    for (int i = 0; i < vertex_size; i++) {
        Vertex* v = contextPool.getVertexByIndex(vertices[i]);
        printf("%d:\t", idx++);
        v->print();
    }
}

void Facet::print_off() {
    printf("OFF\n%ld 1 0\n", vertex_size);
    for (int i = 0; i < vertex_size; i++) {
        Vertex* v = contextPool.getVertexByIndex(vertices[i]);
        v->print();
    }
    printf("%ld\t", vertex_size);
    for (int i = 0; i < vertex_size; i++) {
        printf("%d ", i);
    }
    printf("\n");
}

bool Facet::equal(const Facet& rhs) const {
    if (vertex_size != rhs.vertex_size) {
        return false;
    }

    for (int i = 0; i < vertex_size; i++) {
        Vertex* vertex = contextPool.getVertexByIndex(vertices[i]);

        for (int i = 0; i < rhs.vertex_size; i++) {
            Vertex* vt = contextPool.getVertexByIndex(vertices[i]);
            if (vt == vertex) {
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
    // return vertices.size();
    return vertex_size;
}

// cuda

__device__ Vertex* Facet::getVertexByIndexOnCuda(Vertex* dvertices, int index) {
    if (index == -1) {
        return nullptr;
    }
    return &dvertices[vertices[index]];
}

__device__ Halfedge* Facet::getHalfedgeByIndexOnCuda(Halfedge* dhalfedges, int index) {
    if (index == -1) {
        return nullptr;
    }
    return &dhalfedges[halfedges[index]];
}

__device__ void Facet::addHalfedgeOnCuda(Halfedge* halfedge) {
    halfedges[halfedge_size++] = halfedge->poolId;
}

__device__ void Facet::addHalfedgeOnCuda(int halfedge) {
    halfedges[halfedge_size++] = halfedge;
}

__device__ void Facet::addVertexOnCuda(Vertex* vertex) {
    vertices[vertex_size++] = vertex->poolId;
}

__device__ void Facet::addVertexOnCuda(int vertex) {
    vertices[vertex_size++] = vertex;
}
}  // namespace MCGAL
