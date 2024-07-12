#include <CGAL/IO/OFF.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <fstream>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

namespace SMS = CGAL::Surface_mesh_simplification;

int main() {
    Mesh mesh;
    std::ifstream input("input.off");
    if (!input || !(input >> mesh)) {
        std::cerr << "Error reading input mesh." << std::endl;
        return 1;
    }

    SMS::Count_stop_predicate<Mesh> stop(100);  // 停止条件：仅保留100条边

    SMS::edge_collapse(mesh, stop);

    std::ofstream output("output.off");
    if (!output || !CGAL::IO::write_OFF(output, mesh)) {
        std::cerr << "Error writing output mesh." << std::endl;
        return 1;
    }

    return 0;
}
