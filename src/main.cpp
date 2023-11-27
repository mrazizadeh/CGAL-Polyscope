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
#include <mesh_utilities.h>
#include <mesh_morpho_refine.h>

#include <object_writer.h>
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
std::shared_ptr<ManifoldSurfaceMesh> mesh;
std::shared_ptr<VertexPositionGeometry> geometry;
std::shared_ptr<CGAL::Surface_mesh<Point>> cgalMesh;
// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh;

std::vector<polyscope::SurfaceMesh*> meshVector;
MeshUtilities meshutilities;
MeshRepair meshrepair;
MeshTransform meshtransform;
int refine_count = 0;
// Some algorithm parameters
float param1 = 0.5;
bool  mesh_is_refined = false;
polyscope::SurfaceMesh *refinedMesh2;


// Parameters which we will set in the callback UI for Morphobased Remeshing.
float meanCurvCutOff = 0.05;
float gaussCurvCutOff= 0.5;
float epsilon = 0.25;
float splitThreshold = 1.5;
float collapseThreshold = 0.5;
float timeStep = 0.001;
int numShapeSmoothingIter = 10;
int numSmoothingIters = 5;
int morphoType = 2;
bool findMorphology = false;
char outputFileName[256] = "file";  // Initialize with a default value
static const char* current_item = "Hyperbolic";

// Example computation function -- this one computes and registers a scalar
// quantity
void refineMesh(float param) {
  

  double AverageEdgeLength = meshutilities.averageEdgeLength(cgalMesh);
  std::cout<< "average mesh length= "<< AverageEdgeLength <<std::endl;


  polyscope::warning("Mesh is refined by "+ std::to_string(param)+" average edge length\n The average edge length is = " +
                     std::to_string(AverageEdgeLength));
  meshrepair.isotropicRemeshing(cgalMesh, AverageEdgeLength*param,1);
}



void smoothMesh(float timeStep, int numIters ) {
  meshrepair.shapeSmoothing(cgalMesh, numIters, timeStep );
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {
  ImGui::Text("Shape Refining Using CGAL");
  ImGui::Text("Refining ratio based on current average edge length:");
  ImGui::SliderFloat("Ratio", &param1, 0.1, 4.0);

  if (ImGui::Button("Refine Mesh using CGAL")) {
    cgalMesh = std::make_shared<CGAL::Surface_mesh<K::Point_3>>(meshtransform.toCGAL(mesh, geometry));
    refineMesh(param1);
    auto result = meshtransform.toGC(cgalMesh);
    std::shared_ptr<gcs::ManifoldSurfaceMesh> surfaceMesh = std::move(std::get<0>(result));
    std::shared_ptr<gcs::VertexPositionGeometry> vertexGeometry = std::move(std::get<1>(result));
    std::swap(mesh, surfaceMesh);
    std::swap(geometry, vertexGeometry);
    psMesh = polyscope::registerSurfaceMesh("Mesh", geometry->vertexPositions, 
                              mesh->getFaceVertexList());
    //vertexGeometry->requireVertexPositions();
    //polyscope::SurfaceMesh *refinedMesh;
    //refinedMesh = polyscope::registerSurfaceMesh("Refined Mesh " + std::to_string(refine_count), vertexGeometry->vertexPositions, 
    //                               surfaceMesh->getFaceVertexList());
    //refine_count +=1;
    //writeMeshToOBJ(std::move(surfaceMesh), std::move(vertexGeometry), "refinedMesh"+std::to_string(refine_count)+".obj");
  }

  ImGui::Text("Shape Smoothing Using CGAL:");
  ImGui::InputFloat("Smoothing Timestep", &timeStep);
  ImGui::InputInt("Number of Iterations", &numShapeSmoothingIter);
    if (ImGui::Button("Shape Smoothing using CGAL")) {
    cgalMesh = std::make_shared<CGAL::Surface_mesh<K::Point_3>>(meshtransform.toCGAL(mesh, geometry));
    smoothMesh(timeStep,numShapeSmoothingIter);
    auto result = meshtransform.toGC(cgalMesh);
    std::shared_ptr<gcs::ManifoldSurfaceMesh> surfaceMesh = std::move(std::get<0>(result));
    std::shared_ptr<gcs::VertexPositionGeometry> vertexGeometry = std::move(std::get<1>(result));
    std::swap(mesh, surfaceMesh);
    std::swap(geometry, vertexGeometry);
    psMesh = polyscope::registerSurfaceMesh("Mesh", geometry->vertexPositions, 
                              mesh->getFaceVertexList());
  }
  ImGui::Text("For morphology-based refinement Find Morphologies first:");

  ImGui::InputFloat("Mean Curvature Cutoff", &meanCurvCutOff);
  ImGui::InputFloat("Gaussian Curvature Cutoff", &gaussCurvCutOff);
  if (ImGui::Button("Find Morphologies")) {

  MeshMorphoRefine morphoRefine(mesh, geometry);
  //     if (mesh_is_refined)
  //     {
  //         refinedMesh2->addVertexScalarQuantity("Morphologies", morphoRefine.morphoCategorize(mesh,geometry,meanCurvCutOff,gaussCurvCutOff));
  // }
  //     else{psMesh->addVertexScalarQuantity("Morphologies", morphoRefine.morphoCategorize(mesh,geometry,meanCurvCutOff,gaussCurvCutOff));}
      psMesh->addVertexScalarQuantity("Morphologies", morphoRefine.morphoCategorize(mesh,geometry,meanCurvCutOff,gaussCurvCutOff));
      findMorphology = true;
}
ImGui::Text("Morphologies: Flat sheets = 0, Parabolic = 1,");
ImGui::Text("Hyperbolic = 2, Elliptic = 3");



//if (morphoType != -1)
//{
//    std::cout << current_item << " morphology with the number "<<morphoType<<" is chosen for further refinement" << std::endl;
//}


  if (findMorphology){
      const char* items[] = { "Flat Sheets", "Parabolic", "Hyperbolic", "Elliptic" };
      ImGui::Text("Select the Morphology for Further Refinement");
      if (ImGui::BeginCombo("Morphology Category", current_item)) // The second parameter is the label previewed before opening the combo.
      {
          for (int n = 0; n < IM_ARRAYSIZE(items); n++)
          {
              bool is_selected = (current_item == items[n]); // You can store your selection however you want, outside or inside your objects
              if (ImGui::Selectable(items[n], is_selected))
              {
                  current_item = items[n];
                  if (is_selected)
                      ImGui::SetItemDefaultFocus();   // You may set the initial focus when opening the combo (scrolling + for keyboard navigation support)
              }
          }
          ImGui::EndCombo();
      }
      if (current_item)
      {
          if (strcmp(current_item, "Flat Sheets") == 0)
          {
              morphoType = 0;
          }
          else if (strcmp(current_item, "Parabolic") == 0)
          {
              morphoType = 1;
          }
          else if (strcmp(current_item, "Hyperbolic") == 0)
          {
              morphoType = 2;
          }
          else if (strcmp(current_item, "Elliptic") == 0)
          {
              morphoType = 3;
          }
      }
      ImGui::Text("Morphology-Based Refinement Parameters:");
      ImGui::InputFloat("Epsilon", &epsilon);
      ImGui::InputFloat("Split Threshold", &splitThreshold);
      ImGui::InputFloat("Collapse Threshold", &collapseThreshold);
      ImGui::InputInt("Smoothing Iterations", &numSmoothingIters);

      if (ImGui::Button("Refine Mesh Morpho-Based")) {
          MeshMorphoRefine morphoRefine(mesh, geometry);
          morphoRefine.refineMorphologyBased(morphoType, epsilon, splitThreshold, collapseThreshold, meanCurvCutOff, gaussCurvCutOff, numSmoothingIters);
          psMesh = polyscope::registerSurfaceMesh("Mesh", geometry->vertexPositions, 
                                      mesh->getFaceVertexList());
          mesh_is_refined = true;

      }
  }

  ImGui::Text("Saving Mesh:");
  ImGui::InputText("Output File Name", outputFileName, sizeof(outputFileName));
  if (ImGui::Button("Save Current Mesh")) {
    writeMeshToOBJ(mesh, geometry, std::string(outputFileName) + ".obj");
  }

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

  psMesh = polyscope::registerSurfaceMesh("Mesh", geometry->vertexPositions, 
                              mesh->getFaceVertexList());

  meshVector.push_back(psMesh);


 //transform from cgal to geometry central and in reverse direction
  cgalMesh = std::make_shared<CGAL::Surface_mesh<K::Point_3>>(meshtransform.toCGAL(mesh, geometry));

  polyscope::show();


  return EXIT_SUCCESS;
}
