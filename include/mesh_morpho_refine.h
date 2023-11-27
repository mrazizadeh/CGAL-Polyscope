#pragma once

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/direction_fields.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include <memory>
#include <queue>
using std::cout;
using std::queue;
using namespace geometrycentral;
using namespace geometrycentral::surface;
namespace gc = geometrycentral;
namespace gcs = geometrycentral::surface;
typedef std::shared_ptr<gcs::ManifoldSurfaceMesh> MeshPtr;
typedef std::shared_ptr<gcs::VertexPositionGeometry> GeomPtr;

class MeshMorphoRefine {
public:
    MeshMorphoRefine(const MeshPtr& mesh_,
                     const GeomPtr& geom_);
    void refineMorphologyBased(int refine_morpho_type_, double epsilon_, double split_th_, 
                               double collapse_th_, double mc_, double gc_, int num_iters_);       
    gcs::VertexData<int> morphoCategorize(const MeshPtr& mesh,
            const GeomPtr& geometry,
            double mc,
            double gc) const;
           

    gcs::VertexData<int> morphoCat;
    bool didSplitOrCollapse;    
private:

    bool adjustEdgeLengthsMorphoAdaptive(const MeshPtr& mesh, 
                                         const GeomPtr& geometry, 
                                         double flatLength, double epsilon, double minLength);
    MeshPtr mesh;
    GeomPtr geom;
    double epsilon;
    double mc;
    double gc;
    double initialAverageLength;
    double split_th = 1.5;
    double collapse_th = 0.5;
    int refine_morpho_type = 3;
    const size_t INVALID_IND = std::numeric_limits<size_t>::max();
    int num_iters;
};
