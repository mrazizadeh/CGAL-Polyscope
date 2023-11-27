#include "object_writer.h"
#include "CGALPoly_types.h"
#include <iostream>
#include <fstream>
#include <memory>
void writeMeshToOBJ(const std::shared_ptr<SurfaceMesh> mesh, const std::shared_ptr<VertexPositionGeometry>  geom, std::string output)
{
    using namespace std;
    ofstream outfile;
    outfile.open(output);

    VertexIndices inds = mesh->getVertexIndices();

    // Write all vertices in order
    for (size_t i = 0; i < mesh->nVertices(); i++)
    {
        GCVertex vert = mesh->vertex(i);
        Vector3 pos = geom->inputVertexPositions[vert];
        outfile << "v " << pos.x << " " << pos.y << " " << pos.z << endl;
    }
    
    // // Write area ratios to uvs
    // if (writeAreaRatios)
    // {
    //     for (size_t i = 0; i < mesh->nVertices(); i++)
    //     {
    //         GCVertex vert = mesh->vertex(i);
    //         double r = geomOrig->vertexDualArea(vert) / geom->vertexDualArea(vert);
    //         outfile << "vt " << r << " " << 0 << endl;
    //     }
    // }

    // Write all face indices
    for (GCFace face : mesh->faces())
    {
        outfile << "f ";
        for (GCVertex adjVert : face.adjacentVertices())
        {
            // OBJ is 1-indexed
            int vertInd = inds[adjVert] + 1;
            outfile << vertInd;
            outfile << " ";
        }
        outfile << endl;
    }
    
    outfile << endl;
    outfile.close();
}