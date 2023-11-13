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
typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Surface_mesh<K::Point_3>                        Mesh;
typedef boost::graph_traits<Mesh>::halfedge_descriptor        halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor            edge_descriptor;
typedef boost::graph_traits<Mesh>::vertex_descriptor                 vertex_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace NP = CGAL::parameters;


void merge_vertices(vertex_descriptor v_keep, vertex_descriptor v_rm, Mesh& mesh)
{
  std::cout << "merging vertices " << v_keep << " and " << v_rm << std::endl;
  for(halfedge_descriptor h : CGAL::halfedges_around_target(v_rm, mesh))
    set_target(h, v_keep, mesh); // to ensure that no halfedge points at the deleted vertex
  remove_vertex(v_rm, mesh);
}

struct halfedge2edge
{
  halfedge2edge(const Mesh& m, std::vector<edge_descriptor>& edges)
    : m_mesh(m), m_edges(edges)
  {}
  void operator()(const halfedge_descriptor& h) const
  {
    m_edges.push_back(edge(h, m_mesh));
  }
  const Mesh& m_mesh;
  std::vector<edge_descriptor>& m_edges;
};



int main(int argc, char* argv[])
{


  // Isotropic Remeshing

  const std::string filename = (argc > 1) ? argv[1] : CGAL::data_file_path("meshes/pig.off");
  Mesh mesh;
  if(!PMP::IO::read_polygon_mesh(filename, mesh) || !CGAL::is_triangle_mesh(mesh))
  {
    std::cerr << "Invalid input." << std::endl;
    return 1;
  }
  double target_edge_length = (argc > 2) ? std::stod(std::string(argv[2])) : 6.0;
  unsigned int nb_iter = 3;
  std::cout << "Split border...";
  std::vector<edge_descriptor> border;
  PMP::border_halfedges(faces(mesh), mesh, boost::make_function_output_iterator(halfedge2edge(mesh, border)));
  PMP::split_long_edges(border, target_edge_length, mesh);
  std::cout << "done." << std::endl;
  std::cout << "Start remeshing of " << filename
    << " (" << num_faces(mesh) << " faces)..." << std::endl;
  PMP::isotropic_remeshing(faces(mesh), target_edge_length, mesh,
                           CGAL::parameters::number_of_iterations(nb_iter)
                                            .protect_constraints(true)); //i.e. protect border, here
// Non-Manifold Correction


  int counter = 0;
  for(vertex_descriptor v : vertices(mesh))
  {
    if(PMP::is_non_manifold_vertex(v, mesh))
    {
      std::cout << "vertex " << v << " is non-manifold" << std::endl;
      ++counter;
    }
  }
  std::cout << counter << " non-manifold occurrence(s)" << std::endl;
  // Fix manifoldness by splitting non-manifold vertices
  std::vector<std::vector<vertex_descriptor> > duplicated_vertices;
  std::size_t new_vertices_nb = PMP::duplicate_non_manifold_vertices(mesh,
                                                                     NP::output_iterator(
                                                                       std::back_inserter(duplicated_vertices)));
  std::cout << new_vertices_nb << " vertices have been added to fix mesh manifoldness" << std::endl;
  for(std::size_t i=0; i<duplicated_vertices.size(); ++i)
  {
    std::cout << "Non-manifold vertex " << duplicated_vertices[i].front() << " was fixed by creating";
    for(std::size_t j=1; j<duplicated_vertices[i].size(); ++j)
      std::cout << " " << duplicated_vertices[i][j];
    std::cout << std::endl;
  }

  // Shape smoothing

  const unsigned int nb_iterations = (argc > 3) ? std::atoi(argv[3]) : 10;
  const double time = (argc > 4) ? std::atof(argv[4]) : 1.0;
  std::set<Mesh::Vertex_index> constrained_vertices;
  for(Mesh::Vertex_index v : vertices(mesh))
  {
    if(is_border(v, mesh))
      constrained_vertices.insert(v);
  }
  std::cout << "Constraining: " << constrained_vertices.size() << " border vertices" << std::endl;
  CGAL::Boolean_property_map<std::set<Mesh::Vertex_index> > vcmap(constrained_vertices);
  std::cout << "Smoothing shape... (" << nb_iterations << " iterations)" << std::endl;
  PMP::smooth_shape(mesh, time, CGAL::parameters::number_of_iterations(nb_iterations)
                                                 .vertex_is_constrained_map(vcmap));


  //Write the mesh object

  CGAL::IO::write_polygon_mesh("out.obj", mesh, CGAL::parameters::stream_precision(17));
  std::cout << "Remeshing done." << std::endl;
  return 0;
}