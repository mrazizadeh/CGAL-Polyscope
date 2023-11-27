#pragma once
#include <memory>
#include "CGALPoly_types.h"
using namespace CGALPoly;
using namespace geometrycentral;
using namespace geometrycentral::surface;
void writeMeshToOBJ(const std::shared_ptr<SurfaceMesh> mesh, const std::shared_ptr<VertexPositionGeometry>  geom, std::string output);
