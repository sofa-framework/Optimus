/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ENGINE_TransformStochasticEngine_INL
#define SOFA_COMPONENT_ENGINE_TransformStochasticEngine_INL

#include <sofa/helper/rmath.h> //M_PI
#include <sofa/core/objectmodel/Base.h>
#include <genericComponents/TransformStochasticEngine.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaMeshCollision/RayTriangleIntersection.h>
#include <SofaGeneralMeshCollision/TriangleOctree.h>
#include <cassert>

namespace sofa
{

namespace component
{

namespace engine
{
using helper::ReadAccessor;
using helper::WriteOnlyAccessor;
using helper::vector;
using collision::TriangleOctreeRoot;

template <class DataTypes>
TransformStochasticEngine<DataTypes>::TransformStochasticEngine()
    : f_inputX ( initData (&f_inputX, "input_position", "input array of 3d points") )
    , f_outputX( initData (&f_outputX, "output_position", "output array of 3d points") )
    , translation(initData(&translation, defaulttype::Vector3(0,0,0),"translation", "translation vector ") )
    , rotation(initData(&rotation, defaulttype::Vector3(0,0,0), "rotation", "rotation vector ") )
    , quaternion(initData(&quaternion, defaulttype::Quaternion(0,0,0,1), "quaternion", "rotation quaternion ") )
    , scale(initData(&scale, defaulttype::Vector3(1,1,1),"scale", "scale factor") )
    , inverse(initData(&inverse, false, "inverse", "true to apply inverse transformation"))
    , d_optimParams(initData(&d_optimParams,"optimParams", "build transformation matrix from OptimParams "))
    , d_triangles( initData (&d_triangles, "triangles", "input mesh triangles") )
    , d_quads( initData (&d_quads, "quads", "input mesh quads") )
    , d_normals( initData (&d_normals, "normal", "point normals") )
    , d_stochEstim(initData(&d_stochEstim, true, "stochEstim", "set to True to retrieve transformation matrix from optimParams"))

{
    addInput(&f_inputX);
    addInput(&d_triangles);
    addInput(&d_quads);
    addOutput(&d_normals);
    addInput(&translation);
    addInput(&rotation);
    addInput(&quaternion);
    addInput(&scale);
    addInput(&inverse);
    addInput(&d_optimParams);
    addOutput(&f_outputX);
    setDirtyValue();
}

template <class DataTypes>
void TransformStochasticEngine<DataTypes>::init()
{
}
template <class DataTypes>
void TransformStochasticEngine<DataTypes>::bwdInit()
{
    if((d_triangles.getValue().size()==0) && (d_quads.getValue().size()==0))
        msg_warning(this) << "No input mesh";

    if(f_inputX.getValue().size()==0)
        msg_warning(this) << "No input position";

    if(d_optimParams.getValue().size()!=7)
        msg_warning(this) << "Wrong Size of Stochastic Params. Needs 3 angles, 3 position and 1 scalar to dilate";

}

template <class DataTypes>
void TransformStochasticEngine<DataTypes>::reinit()
{
    update();

}

//Declare a TransformOperation class able to do an operation on a Coord
template <class DataTypes>
struct TransformOperation
{
    virtual ~TransformOperation() {}
    virtual void execute(typename DataTypes::Coord &v) const =0;
};

//*****************************************************************
//Scale Operation
template <class DataTypes>
struct Scale : public TransformOperation<DataTypes>
{
    typedef typename DataTypes::Real Real;
    Scale():sx(0),sy(0),sz(0) {}

    void execute(typename DataTypes::Coord &p) const
    {
        Real x,y,z;
        DataTypes::get(x,y,z,p);
        DataTypes::set(p,x*sx,y*sy,z*sz);
    }

    void configure(const defaulttype::Vector3 &s, bool inverse)
    {
        if (inverse)
        {
            sx=(Real)(1.0/s[0]); sy=(Real)(1.0/s[1]); sz=(Real)(1.0/s[2]);
        }
        else
        {
            sx=(Real)s[0]; sy=(Real)s[1]; sz=(Real)s[2];
        }
    }
private:
    Real sx,sy,sz;
};


//*****************************************************************
//Rotation Operation
template <class DataTypes, int N, bool isVector>
struct RotationSpecialized : public TransformOperation<DataTypes>
{
    typedef typename DataTypes::Real Real;

    void execute(typename DataTypes::Coord &p) const
    {
        defaulttype::Vector3 pos;
        DataTypes::get(pos[0],pos[1],pos[2],p);
        pos=q.rotate(pos);
        DataTypes::set(p,pos[0],pos[1],pos[2]);
    }

    void configure(const defaulttype::Vector3 &r, bool inverse)
    {
        q=helper::Quater<Real>::createQuaterFromEuler( r*(M_PI/180.0));
        if (inverse)
            q = q.inverse();

    }

    void configure(const defaulttype::Quaternion &qi, bool inverse, sofa::core::objectmodel::Base*)
    {
        q=qi;
        if (inverse)
            q = q.inverse();
    }

private:
    defaulttype::Quaternion q;
};


template <class DataTypes>
struct Rotation : public RotationSpecialized<DataTypes, DataTypes::spatial_dimensions, DataTypes::coord_total_size == DataTypes::spatial_dimensions>
{ };

//*****************************************************************
//Translation Operation
template <class DataTypes>
struct Translation : public TransformOperation<DataTypes>
{
    typedef typename DataTypes::Real Real;
    Translation():tx(0),ty(0),tz(0) {}
    void execute(typename DataTypes::Coord &p) const
    {
        Real x,y,z;
        DataTypes::get(x,y,z,p);
        DataTypes::set(p,x+tx,y+ty,z+tz);
    }
    void configure(const defaulttype::Vector3 &t, bool inverse)
    {
        if (inverse)
        {
            tx=(Real)-t[0]; ty=(Real)-t[1]; tz=(Real)-t[2];
        }
        else
        {
            tx=(Real)t[0]; ty=(Real)t[1]; tz=(Real)t[2];
        }
    }
private:
    Real tx,ty,tz;
};


//*****************************************************************
//Functor to apply the operations wanted
template <class DataTypes>
struct Transform
{
    typedef TransformOperation<DataTypes> Op;

    template <class  Operation>
    Operation* add(Operation *op, bool inverse)
    {
        //     Operation *op=new Operation();
        if (inverse)
            list.push_front(op);
        else
            list.push_back(op);
        return op;
    }

    std::list< Op* > &getOperations() {return list;}

    void operator()(typename DataTypes::Coord &v) const
    {
        for (typename std::list< Op* >::const_iterator it=list.begin(); it != list.end() ; ++it)
        {
            (*it)->execute(v);
        }
    }
private:
    std::list< Op* > list;
};


template <class DataTypes>
void TransformStochasticEngine<DataTypes>::update()
{
    const defaulttype::Vector3 &s=scale.getValue();
    const defaulttype::Vector3 &r=defaulttype::Vector3(d_optimParams.getValue()[0],d_optimParams.getValue()[1],d_optimParams.getValue()[2]);
    const defaulttype::Vector3 &t=defaulttype::Vector3(d_optimParams.getValue()[3],d_optimParams.getValue()[4],d_optimParams.getValue()[5]);       const defaulttype::Quaternion &q=quaternion.getValue();


    //Create the object responsible for the transformations
    Transform<DataTypes> transformation;
    const bool inv = inverse.getValue();
    //        if (s != defaulttype::Vector3(1,1,1))
    //            transformation.add(new Scale<DataTypes>, inv)->configure(s, inv);

    //        if (r != defaulttype::Vector3(0,0,0))
    transformation.add(new Rotation<DataTypes>, inv)->configure(r, inv);

    ////        if (q != defaulttype::Quaternion(0,0,0,1))
    //            transformation.add(new Rotation<DataTypes>, inv)->configure(q, inv, this);

    //        if (t != defaulttype::Vector3(0,0,0))
    transformation.add(new Translation<DataTypes>, inv)->configure(t, inv);

    ReadAccessor<Data<VecCoord> > inDilate= f_inputX;
    ReadAccessor<Data<SeqTriangles> > triangles = d_triangles;
    ReadAccessor<Data<SeqQuads> > quads = d_quads;
    Real distance = d_optimParams.getValue()[6];

    cleanDirty();

    WriteOnlyAccessor<Data<VecCoord> > outDilate = f_outputX;

    const int nbp = inDilate.size();
    const int nbt = triangles.size();
    const int nbq = quads.size();

    WriteOnlyAccessor<Data<VecCoord> > normals = d_normals;
    normals.resize(nbp);
    for (int i=0; i<nbp; ++i)
        normals[i].clear();
    for (int i=0; i<nbt; ++i)
    {
        Coord n = cross(inDilate[triangles[i][1]] - inDilate[triangles[i][0]], inDilate[triangles[i][2]] - inDilate[triangles[i][0]]);
        normals[triangles[i][0]] += n;
        normals[triangles[i][1]] += n;
        normals[triangles[i][2]] += n;
    }
    for (int i=0; i<nbq; ++i)
    {
        normals[quads[i][0]] += cross(inDilate[quads[i][1]] - inDilate[quads[i][0]], inDilate[quads[i][3]] - inDilate[quads[i][0]]);
        normals[quads[i][1]] += cross(inDilate[quads[i][2]] - inDilate[quads[i][1]], inDilate[quads[i][0]] - inDilate[quads[i][1]]);
        normals[quads[i][2]] += cross(inDilate[quads[i][3]] - inDilate[quads[i][2]], inDilate[quads[i][1]] - inDilate[quads[i][2]]);
        normals[quads[i][3]] += cross(inDilate[quads[i][0]] - inDilate[quads[i][3]], inDilate[quads[i][2]] - inDilate[quads[i][3]]);
    }
    for (int i=0; i<nbp; ++i)
        normals[i].normalize();

    std::for_each(outDilate.begin(), outDilate.end(), transformation);

    //Deleting operations
    std::list< TransformOperation<DataTypes>* > operations=transformation.getOperations();
    while (!operations.empty())
    {
        delete operations.back();
        operations.pop_back();
    }

    //Set Output
    outDilate.resize(nbp);
    for (int i=0; i<nbp; ++i)
    {
        Real d = distance;
        outDilate[i] = inDilate[i] + normals[i] * d;
    }

}



} // namespace engine

} // namespace component

} // namespace sofa

#endif
