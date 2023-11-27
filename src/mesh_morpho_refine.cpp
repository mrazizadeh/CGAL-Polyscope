#include <mesh_morpho_refine.h>
#include <queue>


inline Vector3 vertexNormal(GeomPtr const &geometry, Vertex v){
    Vector3 norm = Vector3::zero();
    for(Corner c : v.adjacentCorners()){
        norm += geometry->cornerAngle(c) * geometry->faceNormal(c.face());
    }
    //std::cerr<<norm<<std::endl;
    return normalize(norm);
}
inline Vector3 projectToPlane(Vector3 v, Vector3 norm)
{
    return v - norm * dot(norm, v);
}
inline Vector3 findBarycenter(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return (p1 + p2 + p3)/3;
    }

inline Vector3 findBarycenter(GeomPtr const &geometry, Face f)
{
    // retrieve the face's vertices
    int index = 0;
    Vector3 p[3];
    for (Vertex v0 : f.adjacentVertices())
    {
        p[index] = geometry->inputVertexPositions[v0];
        index++;
    }
    return findBarycenter(p[0], p[1], p[2]);
}

inline Vector3 findCircumcenter(Vector3 p1, Vector3 p2, Vector3 p3)
{
    // barycentric coordinates of circumcenter
    double a = (p3 - p2).norm();
    double b = (p3 - p1).norm();
    double c = (p2 - p1).norm();
    double a2 = a * a;
    double b2 = b * b;
    double c2 = c * c;
    Vector3 O{a2 * (b2 + c2 - a2), b2 * (c2 + a2 - b2), c2 * (a2 + b2 - c2)};
    // normalize to sum of 1
    O /= O[0] + O[1] + O[2];
    // change back to space
    return O[0] * p1 + O[1] * p2 + O[2] * p3;
}

inline Vector3 findCircumcenter(GeomPtr const &geometry, Face f)
{
    // retrieve the face's vertices
    int index = 0;
    Vector3 p[3];
    for (Vertex v0 : f.adjacentVertices())
    {
        p[index] = geometry->inputVertexPositions[v0];
        index++;
    }
    return findCircumcenter(p[0], p[1], p[2]);
}

inline bool onBoundary( Face f )
{
    for( Halfedge h : f.adjacentHalfedges() )
    {
        if( !h.twin().isInterior() )
        {
            return true;
        }
    }

    return false;
}
inline bool isDelaunay(GeomPtr const &geometry, Edge e)
{
    float angle1 = geometry->cornerAngle(e.halfedge().next().next().corner());
    float angle2 = geometry->cornerAngle(e.halfedge().twin().next().next().corner());
    return angle1 + angle2 <= PI;
}
inline double diamondAngle(gc::Vector3 a, gc::Vector3 b, gc::Vector3 c, gc::Vector3 d) // dihedral angle at edge a-b
{
    gc::Vector3 n1 = cross(b-a, c-a);
    gc::Vector3 n2 = cross(b-d, a-d);
    return gc::PI-angle(n1, n2);
}
inline bool checkFoldover(gc::Vector3 a, gc::Vector3 b, gc::Vector3 c, gc::Vector3 x, double angle)
{
    return diamondAngle(a, b, c, x) < angle;
}

inline gc::Vector3 edgeMidpoint(MeshPtr const &mesh, GeomPtr const &geometry, gcs::Edge e)
{
    Vector3 endPos1 = geometry->inputVertexPositions[e.halfedge().tailVertex()];
    Vector3 endPos2 = geometry->inputVertexPositions[e.halfedge().tipVertex()];
    return (endPos1+endPos2)/2;
}
inline bool shouldCollapse(MeshPtr const &mesh, GeomPtr const &geometry, gcs::Edge e)
{
    std::vector<Halfedge> toCheck;
    gcs::Vertex v1 = e.halfedge().vertex();
    gcs::Vertex v2 = e.halfedge().twin().vertex();
    gc::Vector3 midpoint = edgeMidpoint(mesh, geometry, e);
    // find (halfedge) link around the edge, starting with those surrounding v1
    gcs::Halfedge he = v1.halfedge();
    gcs::Halfedge st = he;
    do{
        he = he.next();
        if(he.vertex() != v2 && he.next().vertex() != v2){
            toCheck.push_back(he);
        }
        he = he.next().twin();
    }
    while(he != st);
    // v2
    he = v2.halfedge();
    st = he;
    do{
        he = he.next();
        if(he.vertex() != v1 && he.next().vertex() != v1){
            toCheck.push_back(he);
        }
        he = he.next().twin();
    }
    while(he != st);
    
    for(gcs::Halfedge he0 : toCheck){
        gcs::Halfedge heT = he0.twin();
        gcs::Vertex v1 = heT.vertex();
        gcs::Vertex v2 = heT.next().vertex();
        gcs::Vertex v3 = heT.next().next().vertex();
        gc::Vector3 a = geometry->inputVertexPositions[v1];
        gc::Vector3 b = geometry->inputVertexPositions[v2];
        gc::Vector3 c = geometry->inputVertexPositions[v3];
        if(checkFoldover(a, b, c, midpoint, 2)){
            // std::cout<<"prevented foldover"<<std::endl;
            return false;
        }
    }
    return true;
}

inline void fixDelaunay(MeshPtr const &mesh, GeomPtr const &geometry)
        {
            // queue of edges to check if Delaunay
            queue<Edge> toCheck;
            // true if edge is currently in toCheck
            EdgeData<bool> inQueue(*mesh);
            // start with all edges
            for (Edge e : mesh->edges())
            {
                toCheck.push(e);
                inQueue[e] = true;
            }
            // counter and limit for number of flips
            int flipMax = 100 * mesh->nVertices();
            int flipCnt = 0;
            while (!toCheck.empty() && flipCnt < flipMax)
            {
                Edge e = toCheck.front();
                toCheck.pop();
                inQueue[e] = false;
                // if not Delaunay, flip edge and enqueue the surrounding "diamond" edges (if not already)
                if (!e.isBoundary() && !isDelaunay(geometry, e))
                {
                    flipCnt++;
                    Halfedge he = e.halfedge();
                    Halfedge he1 = he.next();
                    Halfedge he2 = he1.next();
                    Halfedge he3 = he.twin().next();
                    Halfedge he4 = he3.next();

                    if (!inQueue[he1.edge()])
                    {
                        toCheck.push(he1.edge());
                        inQueue[he1.edge()] = true;
                    }
                    if (!inQueue[he2.edge()])
                    {
                        toCheck.push(he2.edge());
                        inQueue[he2.edge()] = true;
                    }
                    if (!inQueue[he3.edge()])
                    {
                        toCheck.push(he3.edge());
                        inQueue[he3.edge()] = true;
                    }
                    if (!inQueue[he4.edge()])
                    {
                        toCheck.push(he4.edge());
                        inQueue[he4.edge()] = true;
                    }
                    mesh->flip(e);
                }
            }
        }

void smoothByCircumcenter(MeshPtr const &mesh, GeomPtr const &geometry)
{
    geometry->requireFaceAreas();
    // smoothed vertex positions
    VertexData<Vector3> newVertexPosition(*mesh);
    for (Vertex v : mesh->vertices())
    {
        newVertexPosition[v] = geometry->inputVertexPositions[v]; // default
        if(!v.isBoundary())
        {
            Vector3 updateDirection = Vector3::zero();
            //double totalD = 0;
            for (Face f : v.adjacentFaces())
            {
                // add the center weighted by face area to the update direction
                Vector3 center;

                if( onBoundary(f) )
                {
                    center = findBarycenter(geometry, f);
                }
                else
                {
                    center = findCircumcenter(geometry, f);
                }

                //double D = 1/findFaceTargetL(mesh, geometry, f, 1, 0.1);
                //D = D*D;
                updateDirection += geometry->faceArea(f) * (center - geometry->inputVertexPositions[v]);
                //totalD += geometry->faceArea(f) * D;
            }
            //std::cerr<<updateDirection<<std::endl;
            updateDirection /= (3 * geometry->vertexDualArea(v));
            //updateDirection /= totalD;
            //std::cerr<<"  "<<updateDirection<<std::endl;
            // project update direction to tangent plane
            updateDirection = projectToPlane(updateDirection, vertexNormal(geometry, v));
            Vector3 newPos = geometry->inputVertexPositions[v] + .5 * updateDirection;
            newVertexPosition[v] = newPos;
        }
    }
    // update final vertices
    for (Vertex v : mesh->vertices())
    {
        geometry->inputVertexPositions[v] = newVertexPosition[v];
    }
}




inline double vertexAngleSum(GeomPtr const &geometry, gcs::Vertex v){
        double sum = 0;
        for(gcs::Corner c : v.adjacentCorners()){
            sum += geometry->cornerAngle(c);
        }
        return sum;
}

inline double vertexGaussianCurvature(GeomPtr const &geometry, gcs::Vertex v){
            return 2*PI - vertexAngleSum(geometry, v);
}
        
inline double getSmoothGaussianCurvature(GeomPtr const &geometry, gcs::Vertex v)
{
    double A = geometry->vertexDualArea(v);
    double S = vertexGaussianCurvature(geometry, v);
    double K = S / A;
    return K;
}

inline double getSmoothMeanCurvature(GeomPtr const &geometry, gcs::Vertex v)
{
    double A = geometry->vertexDualArea(v);
    double S = geometry->vertexMeanCurvature(v);
    double K = S / A;
    return K;
}



//From geometry-central MESH and VPG to CGAL
MeshMorphoRefine::MeshMorphoRefine(const MeshPtr& mesh_, const GeomPtr& geom_)
        {
            mesh = std::move(mesh_);
            geom = std::move(geom_);
            geom->requireVertexDualAreas();
            geom->requireVertexMeanCurvatures();

            double sumLength = 0;
            for (gcs::Edge e : mesh->edges())
            {
                sumLength += geom->edgeLength(e);
            }
            initialAverageLength = sumLength / mesh->nEdges();

            std::cout << "Initial average edge length = " << initialAverageLength << std::endl;
        }

void MeshMorphoRefine::refineMorphologyBased(int refine_morpho_type_, double epsilon_, double split_th_, double collapse_th_, double mc_, double gc_, int num_iters_)
{
    epsilon = epsilon_;
    mc = mc_;
    gc = gc_;
    split_th = split_th_;
    collapse_th = collapse_th_;
    refine_morpho_type = refine_morpho_type_;
    num_iters = num_iters_;
    morphoCat = morphoCategorize(mesh, geom, mc, gc);
    double l = initialAverageLength;
    double l_min = initialAverageLength * 0.5;
    didSplitOrCollapse = adjustEdgeLengthsMorphoAdaptive(mesh, geom, l, epsilon, l_min);
        for (int i = 0; i < num_iters; i++)
    {
        smoothByCircumcenter(mesh, geom);
        fixDelaunay(mesh, geom);
    }


}                  

gcs::VertexData<int> MeshMorphoRefine::morphoCategorize(MeshPtr const &mesh, GeomPtr const &geometry, double mc, double gc) const{
            gcs::VertexData<int> totalMorpho(*mesh,0);
            for (gcs::Vertex v : mesh->vertices()){
                double gaussCurv = getSmoothGaussianCurvature(geometry,v);
                double meanCurv = getSmoothMeanCurvature(geometry,v);
                double meanCurvCutoff = mc;
                double meanGaussCutoff = gc;
                if (std::abs(meanCurv) < meanCurvCutoff && std::abs(gaussCurv ) < meanGaussCutoff) {
                    totalMorpho[v] =0;//flat sheet
                }
                else if (std::abs(gaussCurv) < meanGaussCutoff && std::abs(meanCurv) > meanCurvCutoff  ) {
                    totalMorpho[v] =1;//parabolic
                }
                else if (std::abs(gaussCurv) > meanGaussCutoff &&  (gaussCurv) < 0) {
                    totalMorpho[v] =2;//Hyperbolic
                }
                else if (std::abs(gaussCurv) > meanGaussCutoff &&  (gaussCurv) > 0) {
                    totalMorpho[v] =3;//Ellupitic
                }
            }    
            return totalMorpho;
        }

bool MeshMorphoRefine::adjustEdgeLengthsMorphoAdaptive(MeshPtr const &mesh, GeomPtr const &geometry, double flatLength, double epsilon, double minLength)
        {
            bool didSplitOrCollapse = false;
            // queues of edges to CHECK to change
            std::vector<gcs::Edge> toSplit;
            std::vector<gcs::Edge> toCollapse;
            
            for(gcs::Edge e : mesh->edges())
            {
                toSplit.push_back(e);
            }
            
            while(!toSplit.empty())
            {
                gcs::Edge e = toSplit.back();
                toSplit.pop_back();
                double length_e = geometry->edgeLength(e);
                double threshold = (morphoCat[e.halfedge().vertex()]==refine_morpho_type) ? flatLength * epsilon : flatLength;
                if(length_e > minLength && length_e > threshold * split_th )
                {
                    gc::Vector3 newPos = edgeMidpoint(mesh, geometry, e);
                    gcs::Halfedge he = mesh->splitEdgeTriangular(e);
                    didSplitOrCollapse = true;
                    gcs::Vertex newV = he.vertex();
                    geometry->inputVertexPositions[newV] = newPos;                }
                else
                {
                    toCollapse.push_back(e);
                }                
                
            }
            while(!toCollapse.empty())
            {
                gcs::Edge e = toCollapse.back();
                toCollapse.pop_back();
                if(e.halfedge().next().getIndex() != INVALID_IND) // make sure it exists
                {
                    double threshold = (morphoCat[e.halfedge().vertex()]==refine_morpho_type) ? flatLength *epsilon : flatLength;
                    if(geometry->edgeLength(e) < threshold * collapse_th)
                    {
                        gc::Vector3 newPos = edgeMidpoint(mesh, geometry, e);
                        if(shouldCollapse(mesh, geometry, e)) {
                            gcs::Vertex v = mesh->collapseEdgeTriangular(e);
                            didSplitOrCollapse = true;
                            if (v != gcs::Vertex()) {
                                if(!v.isBoundary()) {
                                    geometry->inputVertexPositions[v] = newPos;
                                }
                            }
                        }
                    }
                }
            }
            
            mesh->validateConnectivity();
            mesh->compress();
            geometry->refreshQuantities();
            return didSplitOrCollapse;
        }