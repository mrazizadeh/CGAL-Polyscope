#pragma once

#include "geometrycentral/utilities/vector3.h"
#include "geometrycentral/surface/halfedge_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

#include <Eigen/Core>

namespace CGALPoly
{
    using namespace geometrycentral;
    typedef std::unique_ptr<surface::HalfedgeMesh> MeshUPtr;
    typedef std::unique_ptr<surface::VertexPositionGeometry> GeomUPtr;
    typedef std::shared_ptr<surface::HalfedgeMesh> MeshPtr;
    typedef std::shared_ptr<surface::VertexPositionGeometry> GeomPtr;
    typedef std::shared_ptr<surface::CornerData<Vector2>> UVDataPtr;
    typedef surface::Vertex GCVertex;
    typedef surface::Halfedge GCHalfedge;
    typedef surface::Edge GCEdge;
    typedef surface::Face GCFace;
    typedef surface::Corner GCCorner;

    typedef surface::VertexData<size_t> VertexIndices;
    typedef surface::FaceData<size_t> FaceIndices;

    // if MeshUPtr == std::unique_ptr<surface::HalfedgeMesh>
    inline std::tuple<MeshUPtr, GeomUPtr, std::unique_ptr<surface::CornerData<Vector2>>>
    readParameterizedMesh(std::string filename, std::string type = "")
    {
        return surface::readParameterizedManifoldSurfaceMesh(filename, type);
    };

    inline std::tuple<MeshUPtr, GeomUPtr>
    readMesh(std::string filename, std::string type = "")
    {
        return surface::readManifoldSurfaceMesh(filename, type);
    };

    inline std::tuple<std::unique_ptr<surface::SurfaceMesh>, GeomUPtr>
    readNonManifoldMesh(std::string filename, std::string type = "")
    {
        return surface::readSurfaceMesh(filename, type);
    };

} // namespace biorsurfaces
