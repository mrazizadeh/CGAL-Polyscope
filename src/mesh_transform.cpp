#include <mesh_transform.h>

//From geometry-central MESH and VPG to CGAL
Mesh MeshTransform::toCGAL(const std::unique_ptr<gcs::ManifoldSurfaceMesh>& mesh,
                          const std::unique_ptr<gcs::VertexPositionGeometry>& geometry) const {
    Mesh cgalMesh;
    std::map<gcs::Vertex, Mesh::Vertex_index> vertexMap;
    for (gcs::Vertex v : mesh->vertices()) {
        Point p(geometry->inputVertexPositions[v].x,
                geometry->inputVertexPositions[v].y,
                geometry->inputVertexPositions[v].z);
        vertexMap[v] = cgalMesh.add_vertex(p);
    }

    for (gcs::Face f : mesh->faces()) {
        gcs::Halfedge he = f.halfedge();
        cgalMesh.add_face(std::vector<CGAL::SM_Vertex_index>{vertexMap[he.tailVertex()],
                                                             vertexMap[he.tipVertex()],
                                                             vertexMap[he.next().tipVertex()]});
    }
    return cgalMesh;
}

// From CGAL to GEOMETRY CENTRAL MESH AND VPG
std::tuple<std::unique_ptr<gcs::SurfaceMesh>, std::unique_ptr<gcs::VertexPositionGeometry>>
MeshTransform::toGC(const std::shared_ptr<Mesh>& cgalMesh) const {
    std::vector<std::vector<size_t>> cgalFaces;
    std::map<Mesh::Vertex_index, size_t> vertexIndexMap;  // Map from CGAL vertex index to new index
    size_t currentIndex = 0;
    std::vector<Vector3> cgalVertices;

    for (Mesh::Face_index fd : cgalMesh->faces()) {
        std::vector<size_t> faceIndices;
        Mesh::Halfedge_index hf = cgalMesh->halfedge(fd);

        for (Mesh::Halfedge_index hi : halfedges_around_face(hf, *cgalMesh)) {
            Mesh::Vertex_index vi = target(hi, *cgalMesh);
            // Check if the vertex is already in the map, if not, add it with the next available index
            if (vertexIndexMap.find(vi) == vertexIndexMap.end()) {
                vertexIndexMap[vi] = currentIndex++;
                Point data = cgalMesh->point(vi);
                Vector3 Vec{(float)data.x(), (float)data.y(), (float)data.z()};
                cgalVertices.push_back(Vec);
            }

            faceIndices.push_back(vertexIndexMap[vi]);
        }

        cgalFaces.push_back(faceIndices);
    }

    std::unique_ptr<gcs::SurfaceMesh> surfaceMesh;
    std::unique_ptr<gcs::VertexPositionGeometry> vertexGeometry;
    std::tie(surfaceMesh, vertexGeometry) = makeSurfaceMeshAndGeometry(cgalFaces, cgalVertices);
    return std::make_tuple(std::move(surfaceMesh), std::move(vertexGeometry));
}

