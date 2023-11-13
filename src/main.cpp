#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/direction_fields.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include "args/args.hxx"
#include "imgui.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/draw_surface_mesh.h>
#include <mesh_transform.h>
#include <mesh_repair.h>

#include <iostream>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
using namespace geometrycentral;
using namespace geometrycentral::surface;
namespace gc = geometrycentral;
namespace gcs = geometrycentral::surface;
typedef boost::graph_traits<Mesh>::face_descriptor face_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor  vertex_descriptor;
// == Geometry-central data
std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;
std::shared_ptr<CGAL::Surface_mesh<Point>> cgalMesh;
// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh;

// Some algorithm parameters
float param1 = 42.0;

// Example computation function -- this one computes and registers a scalar
// quantity
void doWork() {
  polyscope::warning("Computing Gaussian curvature.\nalso, parameter value = " +
                     std::to_string(param1));

  geometry->requireVertexGaussianCurvatures();
  psMesh->addVertexScalarQuantity("curvature",
                                  geometry->vertexGaussianCurvatures,
                                  polyscope::DataType::SYMMETRIC);
}

double calculateEdgeLength(const Mesh& mesh, edge_descriptor e) {
    auto point1 = target(halfedge(e, mesh), mesh);
    auto point2 = target(opposite(halfedge(e, mesh), mesh), mesh);
    return CGAL::sqrt(CGAL::squared_distance(mesh.point(point1), mesh.point(point2)));
}
double averageEdgeLength(const Mesh& mesh) {
    double totalLength = 0.0;
    int edgeCount = 0;

    for (const auto& edge : edges(mesh)) {
        totalLength += calculateEdgeLength(mesh, edge);
        ++edgeCount;
    }

    return (edgeCount > 0) ? (totalLength / edgeCount) : 0.0;
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {

  if (ImGui::Button("do work")) {
    doWork();
  }

  ImGui::SliderFloat("param", &param1, 0., 100.);
}

int main(int argc, char **argv) {

  // Configure the argument parser
  args::ArgumentParser parser("CGAL-Polyscope Project");
  args::Positional<std::string> inputFilename(parser, "mesh", "A mesh file.");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help &h) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError &e) {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 1;
  }

  // Make sure a mesh name was given
  if (!inputFilename) {
    std::cerr << "Please specify a mesh file as argument" << std::endl;
    return EXIT_FAILURE;
  }

  // Initialize polyscope
  polyscope::init();

  // Set the callback function
  polyscope::state::userCallback = myCallback;

  // Load mesh
  std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));
  polyscope::registerSurfaceMesh("Original Mesh", geometry->vertexPositions, 
                              mesh->getFaceVertexList());



 //transform from cgal to geometry central and in reverse direction
  MeshTransform meshTransform;
  cgalMesh = std::make_shared<CGAL::Surface_mesh<K::Point_3>>(meshTransform.toCGAL(mesh, geometry));
  MeshRepair meshrepair(cgalMesh);
  double AverageEdgeLength = averageEdgeLength(*cgalMesh);
  std::cout<< "average mesh length= "<< AverageEdgeLength <<std::endl;
  CGAL::draw(*cgalMesh);
  meshrepair.isotropicRemeshing(AverageEdgeLength/2.0,1);
  meshrepair.nonManifoldCorrection();
  CGAL::draw(*cgalMesh);



  auto result = meshTransform.toGC(cgalMesh);
  std::unique_ptr<gcs::SurfaceMesh>& surfaceMesh = std::get<0>(result);
  std::unique_ptr<gcs::VertexPositionGeometry>& vertexGeometry = std::get<1>(result);
  vertexGeometry->requireVertexPositions();
  polyscope::registerSurfaceMesh("Refined Mesh", vertexGeometry->vertexPositions, 
                              surfaceMesh->getFaceVertexList());


  // Give control to the polyscope gui
  polyscope::show();


  return EXIT_SUCCESS;
}
