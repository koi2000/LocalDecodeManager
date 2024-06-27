#include "include/Mesh.cuh"

namespace MCGAL {

Mesh::~Mesh() {
    vertices.clear();
    faces.clear();
}

Facet* Mesh::add_face(std::vector<Vertex*>& vs) {
    Facet* f = contextPool.allocateFaceFromPool(vs);
    faces.push_back(f);
    return f;
}

Facet* Mesh::add_face(Facet* face) {
    faces.push_back(face);
    return face;
}

void Mesh::eraseFacetByPointer(Facet* facet) {
    // for (int i = 0; i < facet->halfedge_size; i++) {
    //     facet->getHalfedgeByIndex(i)->facet_ = -1;
    // }

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

Halfedge* Mesh::split_facet(Halfedge* h, Halfedge* g) {
    Facet* origin = h->facet();
    // early expose
    Facet* fnew = contextPool.allocateFaceFromPool();
    // create new halfedge
    Halfedge* hnew = contextPool.allocateHalfedgeFromPool(h->end_vertex(), g->end_vertex());
    Halfedge* oppo_hnew = contextPool.allocateHalfedgeFromPool(g->end_vertex(), h->end_vertex());
    // this->halfedges.push_back(hnew);
    // this->halfedges.push_back(oppo_hnew);
    // set the opposite
    // set the connect information
    // hnew->setNext(g->next_);
    // oppo_hnew->setNext(h->next_);
    // h->setNext(hnew);
    // g->setNext(oppo_hnew);
    insert_tip(hnew, g);
    insert_tip(hnew->opposite(), h);
    // create new face depend on vertexs
    origin->reset(hnew);
    fnew->flag = origin->flag;
    fnew->processedFlag = origin->processedFlag;
    fnew->removedFlag = origin->removedFlag;
    fnew->reset(oppo_hnew);
    // add halfedge and face to mesh
    this->faces.push_back(fnew);
    return hnew;
}

void Mesh::pre_erase_center_vertex(Halfedge* h, std::vector<int>& encode) {
    Halfedge* g = h->next()->opposite();
    Halfedge* hret = find_prev(h);
    encode.push_back(h->facet()->fid);
    std::vector<int> hids;
    hids.push_back(h->hid);
    hids.push_back(h->opposite()->hid);
    while (g != h) {
        Halfedge* gprev = find_prev(g);
        if (g->facet_ != h->facet_) {
            encode.push_back(g->facet()->fid);
        }
        hids.push_back(g->hid);
        hids.push_back(g->opposite()->hid);
        Halfedge* gnext = g->next()->opposite();
        g = gnext;
    }
    for (int id : hids) {
        encode.push_back(id);
    }
    return;
}

Halfedge* Mesh::erase_center_vertex(Halfedge* h) {
    Halfedge* g = h->next()->opposite();
    Halfedge* hret = find_prev(h);
    // h->facet()->setRemoved();
    while (g != h) {
        Halfedge* gprev = find_prev(g);
        remove_tip(gprev);
        if (g->facet_ != h->facet_) {
            // eraseFacetByPointer(g->facet());
            g->facet()->setRemoved();
        }
        Halfedge* gnext = g->next()->opposite();
        g->vertex()->eraseHalfedgeByPointer(g);
        g->setRemoved();
        g->opposite()->setRemoved();
        g = gnext;
    }
    // h->facet()->setRemoved();
    h->setRemoved();
    h->opposite()->setRemoved();
    // eraseFacetByPointer(h->facet());

    remove_tip(hret);
    h->vertex()->eraseHalfedgeByPointer(h);
    h->end_vertex()->clearHalfedge();
    eraseVertexByPointer(h->end_vertex());
    // Facet* face = contextPool.allocateFaceFromPool(hret);
    h->facet()->reset(hret);
    // faces.push_back(face);
    return hret;
}

Halfedge* Mesh::create_center_vertex(Halfedge* h) {
    // Vertex* vnew = new Vertex();
    Vertex* vnew = contextPool.allocateVertexFromPool();
    this->vertices.push_back(vnew);
    Halfedge* hnew = contextPool.allocateHalfedgeFromPool(h->end_vertex(), vnew);
    Halfedge* oppo_new = contextPool.allocateHalfedgeFromPool(vnew, h->end_vertex());
    // add new halfedge to current mesh and set opposite
    // set the next element
    // now the next of hnew and prev of oppo_new is unknowen
    insert_tip(hnew->opposite(), h);
    Halfedge* g = hnew->opposite()->next();
    std::vector<Halfedge*> origin_around_halfedge;

    Halfedge* hed = hnew;
    while (g->next() != hed) {
        Halfedge* gnew = contextPool.allocateHalfedgeFromPool(g->end_vertex(), vnew);
        Halfedge* oppo_gnew = contextPool.allocateHalfedgeFromPool(vnew, g->end_vertex());
        origin_around_halfedge.push_back(g);
        // gnew->next = hnew->opposite;
        gnew->setNext(hnew->opposite());
        insert_tip(gnew->opposite(), g);

        g = gnew->opposite()->next();
        hnew = gnew;
    }
    // hed->next = hnew->opposite;
    hed->setNext(hnew->opposite());
    h->facet()->reset(h);
    // collect all the halfedge
    for (Halfedge* hit : origin_around_halfedge) {
        Facet* face = contextPool.allocateFaceFromPool(hit);
        this->faces.push_back(face);
    }
    return oppo_new;
}

inline void Mesh::insert_tip(Halfedge* h, Halfedge* v) const {
    // h->next = v->next;
    h->setNext(v->next());
    v->setNext(h->opposite());
    // v->next = h->opposite;
}

Halfedge* Mesh::find_prev(Halfedge* h) const {
    Halfedge* g = h;
    while (g->next() != h)
        g = g->next();
    return g;
}

inline void Mesh::remove_tip(Halfedge* h) const {
    // h->next = h->next->opposite->next;
    h->setNext(h->next()->opposite()->next());
}

Halfedge* Mesh::join_face(Halfedge* h) {
    Halfedge* hprev = find_prev(h);
    Halfedge* gprev = find_prev(h->opposite());
    remove_tip(hprev);
    remove_tip(gprev);
    h->opposite()->setRemoved();
    h->vertex()->eraseHalfedgeByPointer(h);
    h->opposite()->vertex()->eraseHalfedgeByPointer(h->opposite());
    gprev->facet()->setRemoved();
    hprev->facet()->reset(hprev);
    return hprev;
}

// Halfedge* Mesh::join_face(Halfedge* h) {
//     Halfedge* hprev = find_prev(h);
//     Halfedge* gprev = find_prev(h->opposite());
//     remove_tip(hprev);
//     remove_tip(gprev);
//     h->opposite()->setRemoved();
//     // h->vertex()->eraseHalfedgeByPointer(h);
//     // h->opposite()->vertex()->eraseHalfedgeByPointer(h->opposite());
//     gprev->facet()->setRemoved();
//     hprev->facet()->reset(hprev);
//     return hprev;
// }

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

    std::vector<Vertex*> vertices;
    for (std::size_t i = 0; i < nb_vertices; ++i) {
        float x, y, z;
        file >> x >> y >> z;
        Vertex* vt = new Vertex(x, y, z);
        this->vertices.push_back(vt);
        vertices.push_back(vt);
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
        this->add_face(vts);
    }
    vertices.clear();
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

void Mesh::dumpto(std::string path) {
    for (auto fit = faces.begin(); fit != faces.end();) {
        if ((*fit)->isRemoved()) {
            // delete *fit;
            fit = faces.erase(fit);
        } else {
            fit++;
        }
    }

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
        vertex->setId(id++);
    }

    for (Facet* face : this->faces) {
        if (face->isRemoved())
            continue;
        offFile << face->vertex_size << " ";
        Halfedge* hst = MCGAL::contextPool.getHalfedgeByIndex(face->halfedges[0]);
        Halfedge* hed = hst;
        // for (int i = 0; i < face->halfedge_size; i++) {
        //     hst = face->getHalfedgeByIndex(i);
        //     offFile << hst->vertex()->getId() << " ";
        // }

        do {
            offFile << hst->vertex()->getId() << " ";
            hst = hst->next();
        } while (hst != hed);
        offFile << "\n";
    }

    offFile.close();
}

}  // namespace MCGAL