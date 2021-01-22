/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#if !defined(__GNUC__) || (__GNUC__ > 3 || (_GNUC__ == 3 && __GNUC_MINOR__ > 3))
#pragma once
#endif

#include <initOptimusPlugin.h>
#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <sofa/defaulttype/Quat.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>



namespace sofa
{

namespace component
{

namespace engine
{

/**
 * This class transforms the positions of one DataFields into new positions after applying a transformation
This transformation can be either translation, rotation, scale
 */
template <class DataTypes>
class TransformStochasticEngine : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TransformStochasticEngine,DataTypes),core::DataEngine);
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef sofa::core::topology::BaseMeshTopology::SeqTriangles SeqTriangles;
    typedef sofa::core::topology::BaseMeshTopology::SeqQuads SeqQuads;
    typedef typename SeqTriangles::value_type Triangle;
    typedef typename SeqQuads::value_type Quad;

protected:

    TransformStochasticEngine();

    ~TransformStochasticEngine() {}
public:
    void init() override;
    void bwdInit() override;

    void reinit() override;

    void doUpdate() override;

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const TransformStochasticEngine<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

protected:
    Data<VecCoord> f_inputX; // input position
    Data<VecCoord> f_outputX; // ouput position
    Data<defaulttype::Vector3> translation; // translation
    Data<defaulttype::Vector3> rotation; // rotation
    Data<defaulttype::Quaternion> quaternion; // quaternion rotation
    Data<defaulttype::Vector3> scale; // scale
    Data<bool> inverse;
    Data<helper::vector<double>> d_optimParams;
    Data<SeqTriangles> d_triangles; //< input triangles
    Data<SeqQuads> d_quads; //< input quads
    Data<VecCoord> d_normals; //< ouput normals
    Data<bool> d_stochEstim;
};

extern template class SOFA_OPTIMUSPLUGIN_API TransformStochasticEngine<defaulttype::Vec3Types>;


} // namespace engine

} // namespace component

} // namespace sofa

