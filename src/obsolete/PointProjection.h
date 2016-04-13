//
// Class for performing projection of points onto triangular surface
//

#ifndef POINTPROJECTION_H
#define POINTPROJECTION_H

#include <SofaBaseTopology/TriangleSetTopologyContainer.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/helper/vector.h>

namespace sofa
{

/**
 * @brief Projection of points onto 3D triangular surface.
 *
 * @tparam Real Real type to use.
 */

template <class Real>
class PointProjection
{

    public:
        typedef sofa::defaulttype::Vec<2, Real> Vec2;
        typedef sofa::defaulttype::Vec<3, Real> Vec3;

        typedef sofa::defaulttype::Mat<3,3, Real> Mat33;

        typedef sofa::component::topology::TriangleSetTopologyContainer::Edge                   Edge;
        typedef sofa::component::topology::TriangleSetTopologyContainer::TriangleID             Index;
        typedef sofa::component::topology::TriangleSetTopologyContainer::Triangle               Triangle;
        typedef sofa::component::topology::TriangleSetTopologyContainer::TrianglesAroundVertex  TrianglesAroundVertex;
        typedef sofa::component::topology::TriangleSetTopologyContainer::TrianglesAroundEdge    TrianglesAroundEdge;
        typedef sofa::component::topology::TriangleSetTopologyContainer::EdgesInTriangle        EdgesInTriangle;
        typedef sofa::component::topology::TriangleSetTopologyContainer::SeqEdges               SeqEdges;
        typedef sofa::component::topology::TriangleSetTopologyContainer::SeqTriangles           SeqTriangles;

        //typedef sofa::helper::vector<Vec3>      VecVec3;
        typedef sofa::defaulttype::Vec3dTypes::VecCoord VecVec3;
        typedef sofa::defaulttype::Vec3dTypes::Coord Coord;
        typedef sofa::helper::vector<Index>     VecIndex;

        enum { InvalidID = sofa::core::topology::Topology::InvalidID };

        /**
         * @brief Class initialization.
         *
         * @param _topology Associated triangular topology.
         */
        PointProjection(sofa::component::topology::TriangleSetTopologyContainer &_topology) :
            topology(_topology) {}


        /**
         * @brief Find a projection of a point on the surface.
         *
         * @param baryCoords    Barycentric coordinates of projected point.
         * @param triangleID    Triangle on which the point is projected. Or
         *                      InvalidID if projection fails.
         * @param point         Point to project.
         * @param x             Current positions of points in the topology.
         */
        void ProjectPoint(Vec3 &baryCoords, Coord &projectedCoord, Index &triangleID,
            const Vec3 &point, const VecVec3 &x);

        /**
         * @brief Find a projection of a point on the surface of defined set of
         * triangles.
         *
         * @param baryCoords    Barycentric coordinates of projected point.
         * @param triangleID    Triangle on which the point is projected. Or
         *                      InvalidID if projection fails.
         * @param point         Point to project.
         * @param x             Current positions of points in the topology.
         * @param triangleList  Indices of triangle to consider for projection.
         */
        void ProjectPoint(Vec3 &baryCoords, Coord &projectedCoord, Index &triangleID,
            const Vec3 &point, const VecVec3 &x, const VecIndex &triangleList);

        /**
         *
         * Compute barycentric coordinates of a point.
         *
         * Compute barycentric coordinates of point p in triangle whose
         * vertices are a, b and c. If bConstraint is true constraint the
         * coordinates to lie inside the triangle.
         *
         * @param baryCoords    The barycentric coordinates.
         * @param a             Position of first triangle point.
         * @param b             Position of second triangle point.
         * @param c             Position of third triangle point.
         * @param bConstraint   Constraint coordinates to lie inside triangle.
         */
        static void ComputeBaryCoords(
            Vec3 &baryCoords, const Vec3 &p,
            const Vec3 &a, const Vec3 &b, const Vec3 &c, bool bConstraint=true);

        /**
         *
         * Compute barycentric coordinates of a point (2D case).
         *
         * Compute barycentric coordinates of point p in triangle whose
         * vertices are a, b and c. If bConstraint is true constraint the
         * coordinates to lie inside the triangle.
         *
         * @param baryCoords    The barycentric coordinates.
         * @param a             Position of first triangle point.
         * @param b             Position of second triangle point.
         * @param c             Position of third triangle point.
         * @param bConstraint   Constraint coordinates to lie inside triangle.
         */
        static void ComputeBaryCoords(
            Vec3 &baryCoords, const Vec2 &p,
            const Vec2 &a, const Vec2 &b, const Vec2 &c, bool bConstraint=true);

    private:

        sofa::component::topology::TriangleSetTopologyContainer &topology;

    public:

        /**
         * @brief Finds the closest point to a point.
         *
         * @param closestVertex Index of the closest point.
         * @param point         Position of the point.
         * @param inVertices    Positions of points in input set.
         *
         * @return Square of the distance to the closest vertex.
         */
        Real FindClosestPoint(Index& closestVertex,
            const Vec3& point, const VecVec3 &inVertices);

        /**
         * @brief Finds the closest edge to a point.
         *
         * @param closestEdge   Index of the closest edge.
         * @param point         Position of the point.
         * @param inVertices    Positions of points in input set.
         * @param inEdges       LIst of edges.
         *
         * @return Square of the distance to the closest edge.
         */
        Real FindClosestEdge(Index& closestEdge,
            const Vec3& point, const VecVec3 &inVertices,
            const SeqEdges &inEdges);

        /**
         * @brief Finds the closest triangle to a point.
         *
         * @param closestTriangle   Index of the closest triangle.
         * @param point             Position of the point.
         * @param inVertices        Positions of points in input set.
         * @param inEdges           LIst of triangles.
         *
         * @return Square of the distance to the closest triangle.
         */
        Real FindClosestTriangle(Index& closestTriangle,
            const Vec3& point, const VecVec3 &inVertices,
            const SeqTriangles &inTriangles);

    private:
            void ProjectPoint(
                Vec3 &baryCoords, Index &triangleID, Coord &projectedCoord,
                const Vec3 &point, const VecVec3 &x,
                Index closestVertex, Index closestEdge, Index closestTriangle,
                Real minVertex, Real minEdge, Real minTriangle);


};

}

#endif // #ifndef POINTPROJECTION_H
