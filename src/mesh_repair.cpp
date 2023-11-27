#include "mesh_repair.h"


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


const std::shared_ptr<Mesh>& MeshRepair::isotropicRemeshing(const std::shared_ptr<Mesh>& mesh, double target_edge_length, unsigned int nb_iter) {
    splitBorderEdges(mesh, target_edge_length);
    std::cout << "Start remeshing" << std::endl;
    PMP::isotropic_remeshing(faces(*mesh), target_edge_length, *mesh,
                             CGAL::parameters::number_of_iterations(nb_iter)
                                 .protect_constraints(true));
    return mesh;
}


void MeshRepair::nonManifoldCorrection(const std::shared_ptr<Mesh>& mesh) {
    int counter = 0;
    for (vertex_descriptor v : vertices(*mesh)) {
        if (PMP::is_non_manifold_vertex(v, *mesh)) {
            std::cout << "vertex " << v << " is non-manifold" << std::endl;
            ++counter;
        }
    }

    std::vector<std::vector<vertex_descriptor>> duplicated_vertices;
    std::size_t new_vertices_nb =
        PMP::duplicate_non_manifold_vertices(*mesh, NP::output_iterator(std::back_inserter(duplicated_vertices)));
    std::cout << new_vertices_nb << " vertices have been added to fix mesh manifoldness" << std::endl;
    for (std::size_t i = 0; i < duplicated_vertices.size(); ++i) {
        std::cout << "Non-manifold vertex " << duplicated_vertices[i].front() << " was fixed by creating";
        for (std::size_t j = 1; j < duplicated_vertices[i].size(); ++j)
            std::cout << " " << duplicated_vertices[i][j];
        std::cout << std::endl;
    }
    PMP::remove_isolated_vertices(*mesh);
}

void MeshRepair::shapeSmoothing(const std::shared_ptr<Mesh>& mesh, unsigned int nb_iterations, double time) {
    std::set<Mesh::Vertex_index> constrained_vertices;
    for (Mesh::Vertex_index v : vertices(*mesh)) {
        if (is_border(v, *mesh))
            constrained_vertices.insert(v);
    }
    std::cout << "Constraining: " << constrained_vertices.size() << " border vertices" << std::endl;
    CGAL::Boolean_property_map<std::set<Mesh::Vertex_index>> vcmap(constrained_vertices);
    PMP::smooth_shape(*mesh, time, CGAL::parameters::number_of_iterations(nb_iterations)
                                                 .vertex_is_constrained_map(vcmap));

}

void MeshRepair::splitBorderEdges(const std::shared_ptr<Mesh>& mesh, double target_edge_length) {
    std::vector<edge_descriptor> border;
    PMP::border_halfedges(faces(*mesh), *mesh, boost::make_function_output_iterator(halfedge2edge(*mesh, border)));
    PMP::split_long_edges(border, target_edge_length, *mesh);
}

void MeshRepair::fixNonManifoldVertices(const std::shared_ptr<Mesh>& mesh) {
    for (vertex_descriptor v : vertices(*mesh)) {
        if (PMP::is_non_manifold_vertex(v, *mesh)) {
            std::cout << "vertex " << v << " is non-manifold" << std::endl;
        }
    }
}

void MeshRepair::mergeVertices(const std::shared_ptr<Mesh>& mesh, vertex_descriptor v_keep, vertex_descriptor v_rm) {
    std::cout << "merging vertices " << v_keep << " and " << v_rm << std::endl;
    for (halfedge_descriptor h : CGAL::halfedges_around_target(v_rm, *mesh))
        set_target(h, v_keep, *mesh);
    remove_vertex(v_rm, *mesh);
}
