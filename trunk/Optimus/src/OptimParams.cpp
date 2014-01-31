/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

//#define SOFA_COMPONENT_CONTAINER_OPTIMPARAMS_CPP

#
#include <sofa/core/ObjectFactory.h>
#include "OptimParams.inl"
//#include <sofa/helper/accessor.h>

namespace sofa
{

namespace component
{

namespace container
{

using namespace defaulttype;

template<>
void OptimParams<sofa::helper::vector<double> >::init() {
    helper::ReadAccessor<Data<sofa::helper::vector<double> > > initVal = m_initVal;
    size_t nInitVal = initVal.size();

    if (nInitVal == 1 && m_numParams.getValue() > 1) {
        double value = initVal[0];
        nInitVal = m_numParams.getValue();
        helper::WriteAccessor<Data<sofa::helper::vector<double> > > wInitVal = m_initVal;
        wInitVal.wref().resize(nInitVal);
        for (size_t i = 0; i < nInitVal; i++)
            wInitVal[i] = value;
    }


    if (nInitVal != 0) {
        helper::WriteAccessor<Data<sofa::helper::vector<double> > > val = m_val;
        if (val.size() == 0) {
            val.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                val[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::helper::vector<double> > > minVal = m_min;
        if (minVal.size() == 0) {
            minVal.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                minVal[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::helper::vector<double> > > maxVal = m_max;
        if (maxVal.size() == 0) {
            maxVal.resize(nInitVal);
            for (size_t i = 0; i < nInitVal; i++)
                maxVal[i] = initVal[i];
        }

        helper::WriteAccessor<Data<sofa::helper::vector<double> > > stdev = m_stdev;
        //std::cout << "STDEV: " << stdev << std::endl;
        if (stdev.size() == 0) {
            stdev.resize(nInitVal, 0.0);
        } else if (stdev.size() != nInitVal) {
            std::cerr << this->getName() << ": ERROR: |stdev| != |init value|, taking the first member of stdev. " << std::endl;
            //double x=stdev[0];
            stdev.resize(nInitVal);
            for (size_t i = 1; i < nInitVal; i++)
                stdev[i] = stdev[0];

            //std::cout << "STDEV A: " << stdev << std::endl;
            //stdev.resize(nInitVal, x);
            //std::cout << "STDEV B: " << stdev << std::endl;
        }


    }


}

template<>
size_t OptimParams<sofa::helper::vector<double> >::size() {
    helper::ReadAccessor<Data<sofa::helper::vector<double> > > initVal = m_initVal;
    return(initVal.size());
}

SOFA_DECL_CLASS(OptimParams)

// Register in the Factory
int OptimParamsClass = core::RegisterObject("Optimization Parameters")
        #ifndef SOFA_FLOAT
        .add< OptimParams<double> >()
        .add< OptimParams<Vec3d> >()
        .add< OptimParams<Vec2d> >()
        .add< OptimParams<Vec1d> >() // default template
        .add< OptimParams<RigidCoord<3,double> > >()
.add< OptimParams<RigidCoord<2,double> > >()
.add< OptimParams<sofa::helper::vector<double> > >()
#endif
#ifndef SOFA_DOUBLE
.add< OptimParams<float> >(true) // default template
.add< OptimParams<Vec3f> >()
.add< OptimParams<Vec2f> >()
.add< OptimParams<Vec1f> >() // default template
.add< OptimParams<RigidCoord<3,float> > >()
.add< OptimParams<RigidCoord<2,float> > >()
.add< OptimParams<sofa::helper::vector<float> > >()
#endif
;

#ifndef SOFA_FLOAT
template class SOFA_OptimusPlugin_API OptimParams<double>;
template class SOFA_OptimusPlugin_API OptimParams<Vec3d>;
template class SOFA_OptimusPlugin_API OptimParams<Vec2d>;
template class SOFA_OptimusPlugin_API OptimParams<Vec1d>;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<3,double> >;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<2,double> >;
template class SOFA_OptimusPlugin_API OptimParams<sofa::helper::vector<double> >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_OptimusPlugin_API OptimParams<float>;
template class SOFA_OptimusPlugin_API OptimParams<Vec3f>;
template class SOFA_OptimusPlugin_API OptimParams<Vec2f>;
template class SOFA_OptimusPlugin_API OptimParams<Vec1f>;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<3,float> >;
template class SOFA_OptimusPlugin_API OptimParams<RigidCoord<2,float> >;
template class SOFA_OptimusPlugin_API OptimParams<sofa::helper::vector<float> >;
#endif


} // namespace container
} // namespace component
} // namespace sofa




