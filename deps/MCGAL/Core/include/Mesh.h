#ifndef MESH_H
#define MESH_H
#include <vector>
#include "Configuration.h"
#include <string>
#include <algorithm>
#include <stdlib.h>
namespace MCGAL {
class Vertex;
class Halfedge;
class Facet;
class Mesh {
  public:
    // std::unordered_set<Vertex*, Vertex::Hash, Vertex::Equal> vertices;
    std::vector<Vertex*> vertices;
    // std::unordered_set<Vertex*> vertices;
    // std::unordered_set<Halfedge*, Halfedge::Hash, Halfedge::Equal> halfedges;
    // std::unordered_set<Halfedge*> halfedges;
    // std::unordered_set<Facet*> faces;
    std::vector<Facet*> faces;
    std::vector<Halfedge*> halfedges;
    int nb_vertices = 0;
    int nb_faces = 0;
    int nb_edges = 0;

    MCGAL::Vertex** vpool = nullptr;
    MCGAL::Halfedge** hpool = nullptr;
    MCGAL::Facet** fpool = nullptr;
    int vindex = 0;
    int hindex = 0;
    int findex = 0;

  public:
    Mesh() {
        faces.reserve(BUCKET_SIZE);
        vertices.reserve(BUCKET_SIZE);
    }
    ~Mesh();

    // IOS
    bool loadOFF(std::string path);
    void dumpto(std::string path);
    void print();
    std::string to_string();
    Vertex* get_vertex(int vseq = 0);

    // element operating
    Facet* add_face(std::vector<Vertex*>& vs);
    Facet* add_face(Facet* face);
    void eraseFacetByPointer(Facet* facet);
    void eraseVertexByPointer(Vertex* vertex);

    Facet* remove_vertex(Vertex* v);
    Halfedge* merge_edges(Vertex* v);

    /*
     * statistics
     *
     * */
    size_t size_of_vertices() {
        return vertices.size();
    }

    size_t size_of_facets() {
        return faces.size();
    }

    size_t size_of_halfedges() {
        // int count = 0;
        // for (Facet* fit : faces) {
        //     for (Halfedge* hit : fit->halfedges) {
        //         count++;
        //     }
        // }
        // return count;
        return halfedges.size();
    }

    Halfedge* split_facet(Halfedge* h, Halfedge* g);

    Halfedge* create_center_vertex(Halfedge* h);

    inline void close_tip(Halfedge* h, Vertex* v) const;

    inline void insert_tip(Halfedge* h, Halfedge* v) const;

    Halfedge* find_prev(Halfedge* h) const;

    Halfedge* erase_center_vertex(Halfedge* h);

    void set_face_in_face_loop(Halfedge* h, Facet* f) const;

    inline void remove_tip(Halfedge* h) const;

    Halfedge* join_face(Halfedge* h);
};
}  // namespace MCGAL
#endif