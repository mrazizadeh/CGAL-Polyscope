#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/direction_fields.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;

using namespace geometrycentral;
using namespace geometrycentral::surface;
namespace gc = geometrycentral;
namespace gcs = geometrycentral::surface;

class MeshTransform {
public:
    MeshTransform() = default;
    Mesh toCGAL(const std::shared_ptr<gcs::ManifoldSurfaceMesh>& mesh,
                const std::shared_ptr<gcs::VertexPositionGeometry>& geometry) const;

    std::tuple<std::shared_ptr<gcs::ManifoldSurfaceMesh>, std::shared_ptr<gcs::VertexPositionGeometry>> toGC(
        const std::shared_ptr<Mesh>& cgalMesh) const;
};
