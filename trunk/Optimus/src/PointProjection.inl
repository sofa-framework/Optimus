//
// Class for performing projection of points onto triangular surface
//

#include "PointProjection.h"

#include <sofa/helper/rmath.h>

namespace sofa
{

template <class Real>
void PointProjection<Real>::ProjectPoint(Vec3 &baryCoords, Index &triangleID,
    const Vec3 &point, const VecVec3 &x)
{
    Index closestVertex, closestEdge, closestTriangle;
    Real minVertex, minEdge, minTriangle;
    triangleID = InvalidID;

    const SeqEdges &edges = topology.getEdges();
    const SeqTriangles &triangles = topology.getTriangles();

    // Go over vertices
    minVertex = FindClosestPoint(closestVertex, point, x);

    // Go over edges
    minEdge = FindClosestEdge(closestEdge, point, x, edges);

    // Go over triangles
    minTriangle = FindClosestTriangle(closestTriangle, point, x, triangles);

    ProjectPoint(baryCoords, triangleID, point, x,
        closestVertex, closestEdge, closestTriangle,
        minVertex, minEdge, minTriangle);
}

// --------------------------------------------------------------------------------------
template <class Real>
void PointProjection<Real>::ProjectPoint(Vec3 &baryCoords, Index &triangleID,
    const Vec3 &point, const VecVec3 &x, const VecIndex &triangleList)
{
    Index closestVertex=InvalidID, closestEdge=InvalidID, closestTriangle=InvalidID;
    Real minVertex, minEdge=1e12, minTriangle;
    triangleID = InvalidID;

    std::map<Index, bool> subPointsMap;
    VecIndex edgesID;
    SeqEdges edges;
    SeqTriangles triangles(triangleList.size());

    for (unsigned int i=0; i < triangleList.size(); i++) {
        triangles[i] = topology.getTriangle(triangleList[i]);

        EdgesInTriangle elist = topology.getEdgesInTriangle(triangleList[i]);

        edgesID.push_back(elist[0]);
        Edge e = topology.getEdge(elist[0]); 
        edges.push_back(e);
        subPointsMap[e[0]] = true; subPointsMap[e[1]] = true;

        edgesID.push_back(elist[1]);
        e = topology.getEdge(elist[1]); 
        edges.push_back(e);
        subPointsMap[e[0]] = true; subPointsMap[e[1]] = true;

        edgesID.push_back(elist[2]);
        e = topology.getEdge(elist[2]); 
        edges.push_back(e);
        subPointsMap[e[0]] = true; subPointsMap[e[1]] = true;
    }

    VecVec3 subPoints;
    VecIndex subPointsID;
    for (std::map<Index, bool>::const_iterator i=subPointsMap.begin();
        i != subPointsMap.end(); i++) {
        subPoints.push_back(x[i->first]);
        subPointsID.push_back(i->first);
    }

    // Go over vertices
    minVertex = FindClosestPoint(closestVertex, point, subPoints);
    if (closestVertex != InvalidID) {
        closestVertex = subPointsID[closestVertex];
    }

    // Go over edges
    minEdge = FindClosestEdge(closestEdge, point, x, edges);
    if (closestEdge != InvalidID) {
        closestEdge = edgesID[closestEdge];
    }

    // Go over triangles
    minTriangle = FindClosestTriangle(closestTriangle, point, x, triangles);
    if (closestTriangle != InvalidID) {
        closestTriangle = triangleList[closestTriangle];
    }

    ProjectPoint(baryCoords, triangleID, point, x,
        closestVertex, closestEdge, closestTriangle,
        minVertex, minEdge, minTriangle);
}

// -----------------------------------------------------------------------------

// Do some magic to constraint the coordinates inside the triangle
// the requirements are:
//    coef_a, coef_b, coef_c ≥ 0
//    coef_a + coef_b + coef_c = 1
template <class Real>
//void ConstraintBaryCoords(Real &coef_a, Real &coef_b, Real &coef_c)
void ConstraintBaryCoords(sofa::defaulttype::Vec<3, Real> &baryCoords)

{
    if (baryCoords[0] < 0.0) baryCoords[0] = 0.0;
    if (baryCoords[1] < 0.0) baryCoords[1] = 0.0;
    baryCoords[2] = 1.0 - (baryCoords[0] + baryCoords[1]);
    if (baryCoords[2] < 0.0)
    {
        // We have to be carefull so as not to overshoot some other
        // coefficient
        if (baryCoords[0] < -baryCoords[2]/2.0) {
            baryCoords[2] += baryCoords[0];
            baryCoords[1] += baryCoords[2];
            baryCoords[0] = 0.0;
        } else if (baryCoords[1] < -baryCoords[2]/2.0) {
            baryCoords[2] += baryCoords[1];
            baryCoords[0] += baryCoords[2];
            baryCoords[1] = 0.0;
        } else {
            baryCoords[0] += baryCoords[2]/2.0;
            baryCoords[1] += baryCoords[2]/2.0;
        }

        baryCoords[2] = 0.0;
    }
}


// --------------------------------------------------------------------------------------
template <class Real>
void PointProjection<Real>::ComputeBaryCoords(
    Vec3 &baryCoords,
    const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c, bool bConstraint)
{
    const double ZERO = 1e-20;

    Vec3 M = (Vec3) (b-a).cross(c-a);
    double norm2_M = M*(M);

    double coef_a, coef_b, coef_c;

    if(norm2_M < ZERO) // triangle (a,b,c) is flat
    {
        coef_a = (double) (1.0/3.0);
        coef_b = (double) (1.0/3.0);
        coef_c = (double) (1.0 - (coef_a + coef_b));
    }
    else
    {
        Vec3 N =  M/norm2_M;

        coef_a = N*((b-p).cross(c-p));
        coef_b = N*((c-p).cross(a-p));
        if (bConstraint)
        {
            coef_c = 1.0 - (coef_a + coef_b);
        }
        else
        {
            coef_c = N*((a-p).cross(b-p));
        }
    }

    baryCoords[0] = coef_a;
    baryCoords[1] = coef_b;
    baryCoords[2] = coef_c;

    if (bConstraint) {
        ConstraintBaryCoords<Real>(baryCoords);
    }
}

// -----------------------------------------------------------------------------
template <class Real>
void PointProjection<Real>::ComputeBaryCoords(
    Vec3 &baryCoords,
    const Vec2 &p, const Vec2 &a, const Vec2 &b, const Vec2 &c, bool bConstraint)
{
    Mat33 m;
    m(0,0) = 1;    m(0,1) = 1;    m(0,2) = 1;
    m(1,0) = a[0]; m(1,1) = b[0]; m(1,2) = c[0];
    m(2,0) = a[1]; m(2,1) = b[1]; m(2,2) = c[1];

    Mat33 mi;
    mi.invert(m);

    baryCoords = mi * Vec3(1, p[0], p[1]);
    if (bConstraint) {
        ConstraintBaryCoords<Real>(baryCoords);
    }
}

// -----------------------------------------------------------------------------
template <class Real>
Real PointProjection<Real>::FindClosestPoint(Index& closestVertex,
    const Vec3& point, const VecVec3 &inVertices)
{
    Real minimumDistance = 10e12;
    for (unsigned int v=0; v<inVertices.size(); v++)
    {
        Real distance = (inVertices[v] - point).norm2();

        if (distance < minimumDistance)
        {
            // Store the new closest vertex
            closestVertex = v;

            // Updates the minimum's value
            minimumDistance = distance;
        }
    }

    return minimumDistance;
}


// -----------------------------------------------------------------------------
template <class Real>
Real PointProjection<Real>::FindClosestEdge(Index& closestEdge,
    const Vec3& point, const VecVec3 &inVertices, const SeqEdges &inEdges)
{
    Real minimumDistance = 10e12;
    for (unsigned int e=0; e<inEdges.size(); e++)
    {
        Vec3 pointEdge1 = inVertices[ inEdges[e][0] ];
        Vec3 pointEdge2 = inVertices[ inEdges[e][1] ];

        const Vec3 AB = pointEdge2-pointEdge1;
        const Vec3 AP = point-pointEdge1;

        double A;
        double b;
        A = AB*AB;
        b = AP*AB;

        double alpha = b/A;

        // If the point is on the edge
        if (alpha >= 0 && alpha <= 1)
        {
            Vec3 P, Q, PQ;
            P = point;
            Q = pointEdge1 + AB * alpha;
            PQ = Q-P;

            Real distance = PQ.norm2();
            if (distance < minimumDistance)
            {
                // Store the new closest edge
                closestEdge = e;

                // Updates the minimum's value
                minimumDistance = distance;
            }
            }
        }

    return minimumDistance;
}


// -----------------------------------------------------------------------------
template <class Real>
Real PointProjection<Real>::FindClosestTriangle(Index& closestTriangle,
    const Vec3& point, const VecVec3 &inVertices,
    const SeqTriangles &inTriangles)
{
    Real minimumDistance = 10e12;
    for (unsigned int t=0; t<inTriangles.size(); t++)
    {
        Vec3 pointTriangle1 = inVertices[ inTriangles[t][0] ];
        Vec3 pointTriangle2 = inVertices[ inTriangles[t][1] ];
        Vec3 pointTriangle3 = inVertices[ inTriangles[t][2] ];

        const Vec3 AB = pointTriangle2-pointTriangle1;
        const Vec3 AC = pointTriangle3-pointTriangle1;

        Vec3 bary;
        ComputeBaryCoords(
            bary, point,
            pointTriangle1, pointTriangle2, pointTriangle3, false);
        if ((bary[0] < 0.0) || (bary[1] < 0.0) || (bary[2] < 0.0) ||
            (helper::rabs(1.0 - (bary[0] + bary[1] + bary[2])) > 1e-10)) {
            // Point projected onto the plane of the triangle lies outside
            // of the triangle. Some vertex or edge will be more
            // appropriate.
            continue;
        }

        Vec3 N = cross(AB, AC);
        //Real distance = N*point - N*pointTriangle1;
        Real distance = N*(point - pointTriangle1);
        distance = distance*distance / N.norm2();

        if (distance < minimumDistance)
        {
            // Store the new closest triangle
            closestTriangle = t;

            // Updates the minimum's value
            minimumDistance = distance;
        }
    }

    return minimumDistance;
}

// -----------------------------------------------------------------------------
template <class Real>
void PointProjection<Real>::ProjectPoint(
    Vec3 &baryCoords, Index &triangleID,
    const Vec3 &point, const VecVec3 &x,
    Index closestVertex, Index closestEdge, Index closestTriangle,
    Real minVertex, Real minEdge, Real minTriangle)
{
    const SeqTriangles &triangles = topology.getTriangles();

    int which = 2; /* 0 vertex, 1 edge, 2 triangle, 3 nothign*/

    if ((minVertex <= minEdge) && (minVertex <= minTriangle))
    {
        which = 0;
    }
    else if ((minEdge <= minTriangle) && (minEdge <= minVertex))
    {
        which = 1;
    }

    if (which == 0)
    {
        // If it is a vertex, consider one of the triangles attached to it
        TrianglesAroundVertex trianglesAroundVertex = topology.getTrianglesAroundVertex(closestVertex);
        if (trianglesAroundVertex.size() <= 0)
        {
            std::cerr << "No triangles attached to vertex " << closestVertex << std::endl;
            triangleID = InvalidID;
            which = (minEdge <= minTriangle) ? 1 : 2;
        }
        else
        {
            triangleID = trianglesAroundVertex[0];
        }
    }

    if (which == 1)
    {
        // If it is an edge, consider one of the triangles attached to it
        TrianglesAroundEdge trianglesAroundEdge = topology.getTrianglesAroundEdge(closestEdge);
        if (trianglesAroundEdge.size() <= 0)
        {
            std::cerr << "No triangles attached to edge " << closestEdge << std::endl;
            triangleID = InvalidID;
        }
        else
        {
            triangleID = trianglesAroundEdge[0];
        }
    }

    if (which == 2)
    {
        // If it is a triangle, consider it
        triangleID = closestTriangle;
    }

    if (triangleID != InvalidID)
    {
        // Computes barycentric coordinates within the triangle
        ComputeBaryCoords(baryCoords, point,
            x[ triangles[triangleID][0] ],
            x[ triangles[triangleID][1] ],
            x[ triangles[triangleID][2] ]);
    }
}


}
