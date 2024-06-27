#ifndef MESH_H
#define MESH_H
#include "Facet.cuh"
#include "Halfedge.cuh"
#include "Vertex.cuh"
#include "global.cuh"
#include <assert.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace MCGAL {
class Mesh {
  public:
    std::vector<Vertex*> vertices;
    std::vector<Halfedge*> halfedges;
    std::vector<Facet*> faces;
    int nb_vertices = 0;
    int nb_faces = 0;
    int nb_edges = 0;

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
    Facet* remove_vertex(Vertex* v);
    Halfedge* merge_edges(Vertex* v);
    void eraseFacetByPointer(Facet* facet);
    void eraseVertexByPointer(Vertex* vertex);
    void pre_erase_center_vertex(Halfedge* h, std::vector<int>& encode);

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
        int count = 0;
        for (Facet* fit : faces) {
            count += fit->halfedge_size;
        }
        return count;
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