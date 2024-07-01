#include "include/Mesh.h"
#include "include/Facet.h"
#include "include/Global.h"
#include "include/Halfedge.h"
#include "include/Vertex.h"
#include <assert.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
namespace MCGAL {

Mesh::~Mesh() {
    // for (Facet* f : faces) {
    //     delete f;
    // }
    // for (Vertex* p : vertices) {
    //     // assert(p->halfedges.size() == (int)0 && p->opposite_half_edges.size() == 0);
    //     delete p;
    // }
    // for (Halfedge* e : halfedges) {
    //     delete e;
    // }
    // delete[] vpool;
    // delete[] hpool;
    // delete[] fpool;
    vertices.clear();
    faces.clear();
    // halfedges.clear();
}

Facet* Mesh::add_face(std::vector<Vertex*>& vs) {
    Facet* f = new Facet(vs);
    // for (Halfedge* hit : f->halfedges) {
    //     this->halfedges.insert(hit);
    // }
    faces.push_back(f);
    return f;
}

Facet* Mesh::add_face(Facet* face) {
    // for (Halfedge* hit : face->halfedges) {
    //     this->halfedges.insert(hit);
    // }
    faces.push_back(face);
    return face;
}

void Mesh::eraseFacetByPointer(Facet* facet) {
    for (auto it = faces.begin(); it != faces.end();) {
        if ((*it) == facet) {
            faces.erase(it);
            break;
        } else {
            it++;
        }
    }
}

void Mesh::eraseVertexByPointer(Vertex* vertex) {
    for (auto it = vertices.begin(); it != vertices.end();) {
        if ((*it) == vertex) {
            vertices.erase(it);
            break;
        } else {
            it++;
        }
    }
}

MCGAL::Vertex* Mesh::halfedge_collapse(MCGAL::Halfedge* h) {
    h->setRemoved();
    h->opposite->setRemoved();
    // h->opposite->face->setRemoved();
    // h->face->setRemoved();

    MCGAL::Vertex* v1 = h->vertex;
    MCGAL::Vertex* v2 = h->end_vertex;
    v1->setPoint((v1->x() + v2->x()) / 2, (v1->y() + v2->y()) / 2, (v1->z() + v2->z()) / 2);
    // 更改所有v2中边的指向
    for (int i = 0; i < v2->halfedges.size(); i++) {
        MCGAL::Halfedge* hit = v2->halfedges[i];
        if (hit->poolId == h->poolId || hit->opposite->poolId == h->poolId) {
            continue;
        }
        hit->opposite->end_vertex = v1;
        hit->vertex = v1;
        v1->halfedges.push_back(hit);
    }
    // 删除v2持有的边
    v2->halfedges.clear();
    v2->halfedges.shrink_to_fit();
    // 面中若只有三个边，则会发生退化
    if (h->face->facet_degree() == 3) {
        h->face->setRemoved();
        MCGAL::Halfedge* hprev = find_prev(h);
        hprev->opposite->opposite = h->next->opposite;
        h->next->opposite->opposite = hprev->opposite;
        // 退化面中的边需要被移除，并且其对应的顶点也需要移除边
        for (MCGAL::Halfedge* hit : h->face->halfedges) {
            hit->vertex->eraseHalfedgeByPointer(hit);
            hit->setRemoved();
        }
    } else {
        MCGAL::Halfedge* hprev = find_prev(h);
        hprev->next = h->next;
    }
    if (h->opposite->face->facet_degree() == 3) {
        h->opposite->face->setRemoved();
        MCGAL::Halfedge* hprev = find_prev(h->opposite);
        hprev->opposite->opposite = h->opposite->next->opposite;
        h->opposite->next->opposite->opposite = hprev->opposite;
        for (MCGAL::Halfedge* hit : h->opposite->face->halfedges) {
            hit->vertex->eraseHalfedgeByPointer(hit);
            hit->setRemoved();
        }
    } else {
        MCGAL::Halfedge* hprev = find_prev(h->opposite);
        hprev->next = h->next;
    }
    //
    auto newEnd = std::remove_if(v1->halfedges.begin(), v1->halfedges.end(), [](MCGAL::Halfedge* hit) { return hit->isRemoved() || hit->vertex->poolId == hit->end_vertex->poolId; });
    v1->halfedges.resize(std::distance(v1->halfedges.begin(), newEnd));
    for (MCGAL::Halfedge* hit : v1->halfedges) {
        if (hit->opposite == nullptr || hit->opposite->isRemoved()) {
            printf("error");
        }
        hit->face->reset(hit);
        assert(!hit->face->isDegenerate());
        if (hit->face->isDegenerate()) {
            printf("error");
            hit->face->isRemoved();
        }
    }
    return v1;
}

Halfedge* Mesh::split_facet(Halfedge* h, Halfedge* g) {
    Facet* origin = h->face;
    // early expose
    Facet* fnew = MCGAL::contextPool.allocateFaceFromPool();
    // create new halfedge
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(h->end_vertex, g->end_vertex);
    Halfedge* oppo_hnew = MCGAL::contextPool.allocateHalfedgeFromPool(g->end_vertex, h->end_vertex);
    // set the opposite
    // set the connect information
    hnew->next = g->next;
    oppo_hnew->next = h->next;
    h->next = hnew;
    g->next = oppo_hnew;
    // create new face depend on vertexs
    origin->reset(hnew);
    fnew->reset(oppo_hnew);
    fnew->flag = origin->flag;
    fnew->processedFlag = origin->processedFlag;
    fnew->groupId = origin->groupId;
    // fnew->removedFlag = origin->removedFlag;
    // add halfedge and face to mesh
    this->faces.push_back(fnew);
    return hnew;
}

Halfedge* Mesh::erase_center_vertex(Halfedge* h) {
    Halfedge* g = h->next->opposite;
    Halfedge* hret = find_prev(h);

    while (g != h) {
        Halfedge* gprev = find_prev(g);
        remove_tip(gprev);
        if (g->face != h->face) {
            // eraseFacetByPointer(g->facet());
            g->face->setRemoved();
        }
        Halfedge* gnext = g->next->opposite;
        g->vertex->eraseHalfedgeByPointer(g);
        // g->end_vertex->eraseHalfedgeByPointer(g->opposite);
        g->setRemoved();
        g->opposite->setRemoved();
        g = gnext;
    }
    h->setRemoved();
    h->opposite->setRemoved();
    remove_tip(hret);
    h->vertex->eraseHalfedgeByPointer(h);
    h->end_vertex->halfedges.clear();
    h->end_vertex->setRemoved();
    // eraseVertexByPointer(h->end_vertex);
    h->face->reset(hret);
    return hret;
}

Halfedge* Mesh::create_center_vertex(Halfedge* h) {
    // Vertex* vnew = new Vertex();
    Vertex* vnew = MCGAL::contextPool.allocateVertexFromPool();
    this->vertices.push_back(vnew);
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(h->end_vertex, vnew);
    Halfedge* oppo_new = MCGAL::contextPool.allocateHalfedgeFromPool(vnew, h->end_vertex);
    // add new halfedge to current mesh and set opposite
    // set the next element
    // now the next of hnew and prev of oppo_new is unknowen
    insert_tip(hnew->opposite, h);
    Halfedge* g = hnew->opposite->next;
    std::vector<Halfedge*> origin_around_halfedge;

    Halfedge* hed = hnew;
    while (g->next != hed) {
        Halfedge* gnew = MCGAL::contextPool.allocateHalfedgeFromPool(g->end_vertex, vnew);
        Halfedge* oppo_gnew = MCGAL::contextPool.allocateHalfedgeFromPool(vnew, g->end_vertex);
        origin_around_halfedge.push_back(g);
        gnew->next = hnew->opposite;
        insert_tip(gnew->opposite, g);

        g = gnew->opposite->next;
        hnew = gnew;
    }
    hed->next = hnew->opposite;
    h->face->reset(h);
    // collect all the halfedge
    for (Halfedge* hit : origin_around_halfedge) {
        Facet* face = MCGAL::contextPool.allocateFaceFromPool(hit);
        this->faces.push_back(face);
    }
    return oppo_new;
}

inline void Mesh::close_tip(Halfedge* h, Vertex* v) const {
    h->next = h->opposite;
    h->vertex = v;
}

inline void Mesh::insert_tip(Halfedge* h, Halfedge* v) const {
    h->next = v->next;
    v->next = h->opposite;
}

Halfedge* Mesh::find_prev(Halfedge* h) const {
    Halfedge* g = h;
    while (g->next != h) {
        g = g->next;
    }
    return g;
}

void Mesh::set_face_in_face_loop(Halfedge* h, Facet* f) const {
    f->halfedges.clear();
    f->vertices.clear();
    Halfedge* end = h;
    do {
        h->face = f;
        f->halfedges.push_back(h);
        f->vertices.push_back(h->vertex);
        h = h->next;
    } while (h != end);
}

inline void Mesh::remove_tip(Halfedge* h) const {
    h->next = h->next->opposite->next;
}

Halfedge* Mesh::join_face(Halfedge* h) {
    Halfedge* hprev = find_prev(h);
    Halfedge* gprev = find_prev(h->opposite);
    remove_tip(hprev);
    remove_tip(gprev);
    h->opposite->setRemoved();
    h->setRemoved();

    // h->vertex->halfedges.erase(h);
    for (auto it = h->vertex->halfedges.begin(); it != h->vertex->halfedges.end(); it++) {
        if ((*it) == h) {
            h->vertex->halfedges.erase(it);
            break;
        }
    }
    for (auto it = h->opposite->vertex->halfedges.begin(); it != h->opposite->vertex->halfedges.end(); it++) {
        if ((*it) == h->opposite) {
            h->opposite->vertex->halfedges.erase(it);
            break;
        }
    }

    // h->opposite->vertex->halfedges.erase(h->opposite);
    // this->faces.erase(gprev->face);
    gprev->face->setRemoved();
    // delete gprev->face;
    int size = hprev->face->halfedges.size() + gprev->face->halfedges.size() - 2;
    hprev->face->reset(hprev);
    return hprev;
}

bool Mesh::loadOFF(std::string path) {
    std::ifstream fp(path);
    if (!fp.is_open()) {
        std::cerr << "Error: Unable to open file " << path << std::endl;
        return false;
    }

    std::stringstream file;
    file << fp.rdbuf();  // Read the entire file content into a stringstream

    std::string format;
    file >> format >> nb_vertices >> nb_faces >> nb_edges;

    if (format != "OFF") {
        std::cerr << "Error: Invalid OFF file format" << std::endl;
        return false;
    }

    // std::vector<Vertex*> vertices;
    for (int i = 0; i < nb_vertices; ++i) {
        float x, y, z;
        file >> x >> y >> z;
        Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(x, y, z, i);
        // Vertex* vt = new Vertex(x, y, z);
        this->vertices.push_back(vt);
    }

    for (int i = 0; i < nb_faces; ++i) {
        int num_face_vertices;
        file >> num_face_vertices;
        std::vector<Vertex*> vts;
        for (int j = 0; j < num_face_vertices; ++j) {
            int vertex_index;
            file >> vertex_index;
            vts.push_back(vertices[vertex_index]);
        }
        MCGAL::Facet* fit = MCGAL::contextPool.allocateFaceFromPool(vts);
        this->faces.push_back(fit);
    }
    // vertices.clear();
    fp.close();
    return true;
}

std::istream& operator>>(std::istream& input, Mesh& mesh) {
    std::string format;
    // read off header
    input >> format >> mesh.nb_vertices >> mesh.nb_faces >> mesh.nb_edges;
    if (format != "OFF") {
        std::cerr << "Error: Invalid OFF file format" << std::endl;
    }

    // vector used to create face
    std::vector<Vertex*> vertices;
    // add vertex into Mesh
    for (std::size_t i = 0; i < mesh.nb_vertices; ++i) {
        float x, y, z;
        input >> x >> y >> z;
        Vertex* vt = new Vertex(x, y, z);
        vt->setVid(i);
        mesh.vertices.push_back(vt);
        vertices.push_back(vt);
    }

    // write face into mesh
    for (int i = 0; i < mesh.nb_faces; ++i) {
        int num_face_vertices;
        input >> num_face_vertices;
        // std::vector<Facet*> faces;
        std::vector<Vertex*> vts;

        for (int j = 0; j < num_face_vertices; ++j) {
            int vertex_index;
            input >> vertex_index;
            vts.push_back(vertices[vertex_index]);
        }
        Facet* face = mesh.add_face(vts);
        // for (Halfedge* halfedge : face->halfedges) {
        //     mesh.halfedges.insert(halfedge);
        // }
    }
    // clear vector
    vertices.clear();
    return input;
}

inline bool isFacetRemovable(MCGAL::Facet* fit) {
    return fit->isRemoved();
}

void Mesh::dumpto(std::string path) {
    auto newEnd = std::remove_if(faces.begin(), faces.end(), isFacetRemovable);
    faces.resize(std::distance(faces.begin(), newEnd));

    std::ofstream offFile(path);
    if (!offFile.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return;
    }
    // write header
    offFile << "OFF\n";
    offFile << this->vertices.size() << " " << this->faces.size() << " 0\n";
    offFile << "\n";
    // write vertex
    int id = 0;
    for (Vertex* vertex : this->vertices) {
        offFile << vertex->x() << " " << vertex->y() << " " << vertex->z() << "\n";
        vertex->setVid(id++);
    }
    std::vector<std::vector<int>> colors = {
        {0, 0, 0},        // Black
        {255, 0, 0},      // Red
        {0, 255, 0},      // Green
        {0, 0, 255},      // Blue
        {255, 255, 0},    // Yellow
        {255, 0, 255},    // Magenta
        {0, 255, 255},    // Cyan
        {128, 128, 128},  // Gray
        {255, 165, 0},    // Orange
        {128, 0, 128},    // Purple
        {64, 224, 208},   // Turquoise
        {255, 250, 205},  // Lemon Yellow
        {128, 0, 0},      // Maroon
        {127, 255, 212},  // Aquamarine
        {75, 0, 130},     // Indigo
        {255, 160, 122},  // Light Salmon
        {32, 178, 170},   // Light Sea Green
        {135, 206, 250},  // Light Sky Blue
        {119, 136, 153},  // Light Slate Gray
        {176, 196, 222},  // Light Steel Blue
        {255, 255, 224},  // Light Yellow
        {250, 128, 114},  // Salmon
        {106, 90, 205},   // Slate Blue
        {112, 128, 144},  // Slate Gray
        {0, 0, 128},      // Navy
        {189, 183, 107},  // Dark Khaki
        {153, 50, 204},   // Dark Orchid
        {205, 133, 63},   // Peru
        {128, 128, 0},    // Olive
        {160, 82, 45},    // Sienna
        {102, 205, 170},  // Medium Aquamarine
        {50, 205, 50},    // Lime Green
        {70, 130, 180},   // Steel Blue
        {210, 105, 30},   // Chocolate
        {154, 205, 50},   // Yellow Green
        {219, 112, 147},  // Pale Violet Red
        {173, 255, 47},   // Green Yellow
        {240, 255, 255},  // Azure
        {255, 127, 80},   // Coral
        {238, 130, 238},  // Violet
        {75, 0, 130},     // Indigo
        {255, 105, 180},  // Hot Pink
        {0, 250, 154},    // Medium Spring Green
        {0, 191, 255},    // Deep Sky Blue
        {255, 99, 71},    // Tomato
        {245, 222, 179},  // Wheat
        {255, 228, 196},  // Bisque
        {255, 215, 0},    // Gold
        {139, 69, 19},    // Saddle Brown
        {255, 228, 181},  // Moccasin
        {152, 251, 152},  // Pale Green
        {238, 232, 170},  // Pale Goldenrod
        {255, 140, 0},    // Dark Orange
        {255, 140, 105},  // Light Salmon
        {0, 128, 128},    // Teal
        {255, 228, 225},  // Blanched Almond
        {192, 192, 192},  // Silver
        {0, 0, 139},      // Dark Blue
        {173, 216, 230},  // Light Blue
        {0, 255, 255},    // Aqua
        {139, 0, 139},    // Dark Magenta
        {255, 250, 250},  // Snow
        {245, 245, 220},  // Beige
        {0, 255, 127},    // Spring Green
        {255, 105, 180},  // Hot Pink
        {255, 255, 255}   // White
    };
    for (Facet* face : this->faces) {
        if (face->isRemoved())
            continue;
        offFile << face->vertices.size() << " ";
        Halfedge* hst = *face->halfedges.begin();
        Halfedge* hed = *face->halfedges.begin();
        do {
            offFile << hst->vertex->getVid() << " ";
            hst = hst->next;
        } while (hst != hed);

        if (face->groupId >= 0) {
            offFile << colors[face->groupId][0] << " " << colors[face->groupId][1] << " " << colors[face->groupId][2] << " ";
        }
        // else if(face->groupId == -1) {
        //     std::cout << 1 << std::endl;
        //     offFile << 255 << " " << 255 << " " << 255 << " ";
        // }
        // if (face->groupId >= 0 && face->groupId < 10) {
        //     offFile << colors[face->level][0] << " " << colors[face->level][1] << " " << colors[face->level][2] << " ";
        // } else {
        //     offFile << 255 << " " << 255 << " " << 255 << " ";
        // }
        // if (face->level <10) {
        //     offFile << colors[face->level][0] << " " << colors[face->level][1] << " " << colors[face->level][2] << " ";
        // } else {
        //     offFile << 255 << " " << 255 << " " << 255 << " ";
        // }
        offFile << "\n";
    }
    offFile.close();
}

}  // namespace MCGAL