#include "include/Halfedge.cuh"
#include "include/global.cuh"
namespace MCGAL {
// create a new half edge, setup the opposite of this half edge if needed
Halfedge::Halfedge(Vertex* v1, Vertex* v2) {
    vertex_ = v1->poolId;
    end_vertex_ = v2->poolId;

    vertex()->addHalfedge(this);
    // in case this is the second half edge
    for (int i = 0; i < v2->halfedges_size; i++) {
        Halfedge* h = v2->getHalfedgeByIndex(i);
        if (h->end_vertex() == v1) {
            if (h->opposite()) {
                printf("create half edge:\n");
                v1->print();
                v2->print();
                h->opposite()->facet()->print_off();
            }
            assert(h->opposite() == NULL);
            // h->opposite = this;
            // this->opposite = h;
            h->setOpposite(this);
            this->setOpposite(h);
            break;
        }
    }
}

void Halfedge::reset(Vertex* v1, Vertex* v2) {
    vertex_ = v1->poolId;
    end_vertex_ = v2->poolId;
    // vertex()->halfedges.push_back(this);
    vertex()->addHalfedge(this);
    // end_vertex->opposite_half_edges.insert(this);
    // in case this is the second half edge
    // for (Halfedge* h : v2->halfedges) {
    for (int i = 0; i < v2->halfedges_size; i++) {
        Halfedge* h = contextPool.getHalfedgeByIndex(v2->halfedges[i]);
        if (h->end_vertex() == v1) {
            if (h->opposite()) {
                printf("create half edge:\n");
                v1->print();
                v2->print();
                h->opposite()->facet()->print_off();
            }
            assert(h->opposite() == NULL);
            // h->opposite = this;
            // this->opposite = h;
            h->setOpposite(this);
            this->setOpposite(h);
            break;
        }
    }
}

Halfedge::~Halfedge() {}

// cpu
Vertex* Halfedge::vertex() {
    if (vertex_ == -1) {
        return nullptr;
    }
    return contextPool.getVertexByIndex(vertex_);
};

Vertex* Halfedge::end_vertex() {
    if (end_vertex_ == -1) {
        return nullptr;
    }
    return contextPool.getVertexByIndex(end_vertex_);
};
Facet* Halfedge::facet() {
    if (facet_ == -1) {
        return nullptr;
    }
    return contextPool.getFacetByIndex(facet_);
};

Halfedge* Halfedge::opposite() {
    if (opposite_ == -1) {
        return nullptr;
    }
    return contextPool.getHalfedgeByIndex(opposite_);
};

Halfedge* Halfedge::next() {
    if (next_ == -1) {
        return nullptr;
    }
    return contextPool.getHalfedgeByIndex(next_);
};

void Halfedge::setOpposite(Halfedge* opposite) {
    opposite_ = opposite->poolId;
};

void Halfedge::setOpposite(int opposite) {
    opposite_ = opposite;
};

void Halfedge::setNext(Halfedge* next) {
    next_ = next->poolId;
};

void Halfedge::setNext(int next) {
    next_ = next;
};

void Halfedge::setFacet(Facet* facet) {
    facet_ = facet->poolId;
};

void Halfedge::setFacet(int facet) {
    facet_ = facet;
};

// cuda
__device__ void Halfedge::setOppositeOnCuda(Halfedge* opposite) {
    opposite_ = opposite->poolId;
}

__device__ void Halfedge::setOppositeOnCuda(int opposite) {
    opposite_ = opposite;
}

__device__ void Halfedge::setNextOnCuda(Halfedge* next) {
    next_ = next->poolId;
}
__device__ void Halfedge::setNextOnCuda(int next) {
    next_ = next;
}

__device__ void Halfedge::setFacetOnCuda(Facet* facet) {
    facet_ = facet->poolId;
}
__device__ void Halfedge::setFacetOnCuda(int facet) {
    facet_ = facet;
}

__device__ Vertex* Halfedge::dvertex(Vertex* vertices) {
    if (vertex_ == -1) {
        return nullptr;
    }
    return &vertices[vertex_];
}

__device__ Vertex* Halfedge::dend_vertex(Vertex* vertices) {
    if (end_vertex_ == -1) {
        return nullptr;
    }
    return &vertices[end_vertex_];
}

__device__ Facet* Halfedge::dfacet(Facet* facets) {
    if (facet_ == -1) {
        printf("facet is null\n");
        return nullptr;
    }
    return &facets[facet_];
}

__device__ Halfedge* Halfedge::dopposite(Halfedge* halfedges) {
    if (opposite_ == -1) {
        printf("opposite is null\n");
        return nullptr;
    }
    return &halfedges[opposite_];
}

__device__ Halfedge* Halfedge::dnext(Halfedge* halfedges) {
    if (next_ == -1) {
        printf("next is null\n");
        return nullptr;
    }
    return &halfedges[next_];
}

__device__ void Halfedge::resetOnCuda(int v1, int v2) {
    vertex_ = v1;
    end_vertex_ = v2;
}

__device__ void Halfedge::resetOnCuda(Vertex* v1, Vertex* v2) {
    vertex_ = v1->poolId;
    end_vertex_ = v2->poolId;
    // vertices[vertex_].addHalfedgeOnCuda(this->poolId);
    // for (int i = 0; i < v2->halfedges_size; i++) {
    //     Halfedge* h = &halfedges[v2->halfedges[i]];
    //     if (h->dend_vertex(vertices)->poolId == v1->poolId) {
    //         if (h->dopposite(halfedges)) {
    //             printf("create half edge:\n");
    //         }
    //         assert(h->dopposite(halfedges) == NULL);
    //         // h->opposite = this;
    //         // this->opposite = h;
    //         h->setOppositeOnCuda(this);
    //         this->setOppositeOnCuda(h);
    //         break;
    //     }
    // }
}
}  // namespace MCGAL
