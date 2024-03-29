#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <boost/iterator/function_output_iterator.hpp>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/smooth_shape.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> Mesh;
typedef boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace NP = CGAL::parameters;

class MeshRepair {
public:
    MeshRepair() = default;  // Change inputMesh to shared pointer

    const std::shared_ptr<Mesh>& isotropicRemeshing(const std::shared_ptr<Mesh>& mesh, double target_edge_length, unsigned int nb_iter);

    void nonManifoldCorrection(const std::shared_ptr<Mesh>& mesh);

    void shapeSmoothing(const std::shared_ptr<Mesh>& mesh, unsigned int nb_iterations, double time);


private:

    void splitBorderEdges(const std::shared_ptr<Mesh>& mesh, double target_edge_length);

    void fixNonManifoldVertices(const std::shared_ptr<Mesh>& mesh);

    void mergeVertices(const std::shared_ptr<Mesh>& mesh, vertex_descriptor v_keep, vertex_descriptor v_rm);
};