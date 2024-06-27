#include "include/Halfedge.h"
#include "include/Facet.h"
#include "include/Vertex.h"

namespace MCGAL {

// create a new half edge, setup the opposite of this half edge if needed
Halfedge::Halfedge(Vertex* v1, Vertex* v2) {
    vertex = v1;
    end_vertex = v2;
    vertex->halfedges.push_back(this);
    // end_vertex->opposite_half_edges.insert(this);
    // in case this is the second half edge
    for (Halfedge* h : v2->halfedges) {
        if (h->end_vertex == v1) {
            if (h->opposite) {
                printf("create half edge:\n");
                v1->print();
                v2->print();
                h->opposite->face->print_off();
            }
            assert(h->opposite == NULL);
            h->opposite = this;
            this->opposite = h;
            break;
        }
    }
}

void Halfedge::reset(Vertex* v1, Vertex* v2) {
    vertex = v1;
    end_vertex = v2;
    vertex->halfedges.push_back(this);
    // end_vertex->opposite_half_edges.insert(this);
    // in case this is the second half edge
    for (Halfedge* h : v2->halfedges) {
        if (h->end_vertex == v1) {
            if (h->opposite) {
                printf("create half edge:\n");
                v1->print();
                v2->print();
                h->opposite->face->print_off();
            }
            assert(h->opposite == NULL);
            h->opposite = this;
            this->opposite = h;
            break;
        }
    }
}

void Halfedge::setVertex(Vertex* v1, Vertex* v2) {
    vertex = v1;
    end_vertex = v2;
}

Halfedge::~Halfedge() {
    // reset the opposite;
    if (opposite != NULL) {
        assert(opposite->opposite = this);
        opposite->opposite = NULL;
    }

    // detach from the vertices
    // assert(vertex && end_vertex);
    // assert(vertex->halfedges.find(this) != vertex->halfedges.end());
    // assert(end_vertex->opposite_half_edges.find(this) != end_vertex->opposite_half_edges.end());
    // if (vertex->halfedges.find(this) != vertex->halfedges.end()) {
    //     vertex->halfedges.erase(this);
    // }
    // if (end_vertex->opposite_half_edges.find(this) != end_vertex->opposite_half_edges.end()) {
    //     end_vertex->opposite_half_edges.erase(this);
    // }
}
}  // namespace MCGAL
