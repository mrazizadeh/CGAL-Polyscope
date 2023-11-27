#include "mesh_utilities.h"


double MeshUtilities::calculateEdgeLength(const std::shared_ptr<Mesh>& mesh, edge_descriptor e) {
    auto point1 = target(halfedge(e, *mesh), *mesh);
    auto point2 = target(opposite(halfedge(e, *mesh), *mesh), *mesh);
    return CGAL::sqrt(CGAL::squared_distance(mesh->point(point1), mesh->point(point2)));
}
double MeshUtilities::averageEdgeLength(const std::shared_ptr<Mesh>& mesh) {
    double totalLength = 0.0;
    int edgeCount = 0;

    for (const auto& edge : edges(*mesh)) {
        totalLength += calculateEdgeLength(mesh, edge);
        ++edgeCount;
    }

    return (edgeCount > 0) ? (totalLength / edgeCount) : 0.0;
}

void MeshUtilities::writeMesh(const std::shared_ptr<Mesh>& mesh, const std::string& filename) const {
    CGAL::IO::write_polygon_mesh(filename, *mesh, CGAL::parameters::stream_precision(17));
    std::cout << "Mesh is written to file." << std::endl;
}
