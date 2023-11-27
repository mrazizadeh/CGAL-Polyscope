#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor  vertex_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor  edge_descriptor;

// == Geometry-central data

class MeshUtilities {
public:
    MeshUtilities() = default;
    double calculateEdgeLength(const std::shared_ptr<Mesh>& mesh, edge_descriptor e);
    double averageEdgeLength(const std::shared_ptr<Mesh>& mesh);
    void writeMesh(const std::shared_ptr<Mesh>& mesh, const std::string& filename) const;

};
