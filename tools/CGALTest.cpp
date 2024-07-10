#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Mid_edge_collapse_cost.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

#include <fstream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef CGAL::Surface_mesh_simplification::Edge_collapse<Polyhedron> ECF;
typedef CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<ECF> Stop_predicate;
typedef CGAL::Surface_mesh_simplification::Mid_edge_collapse_cost<Kernel, HalfedgeDS> Cost_policy;

int main() {
    Polyhedron mesh;

    // Read mesh from file.
    if (!CGAL::IO::read_OFF("input.off", mesh)) {
        std::cerr << "Not a valid OFF file." << std::endl;
        return 1;
    }

    // Set up the edge collapse function object and policies.
    ECF ecf(mesh);
    Cost_policy cost;
    Stop_predicate sp(0.5);  // Keep 50% of edges.

    // Simplify the mesh.
    while (ecf.number_of_edges() > sp.min_edges() && ecfcollapse(cost)) {
        ecf.collapse(ecf.edge_with_min_cost(cost));
    }

    // Write the simplified mesh to a file.
    CGAL::IO::write_OFF("output.off", mesh);

    return 0;
}