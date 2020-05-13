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

#include <fstream>
#include <sofa/core/ObjectFactory.h>
#include "MatrixCovariance.inl"

namespace sofa
{

namespace component
{

namespace container
{


/// DECLARATIONS

SOFA_DECL_CLASS(MatrixCovariance)

// Register in the Factory
int MatrixCovarianceClass = core::RegisterObject("Covariance data")
        #ifndef SOFA_FLOAT
        .add< MatrixCovariance<double> >()

        #endif
        #ifndef SOFA_DOUBLE
        #endif
        ;

#ifndef SOFA_FLOAT
template class SOFA_OPTIMUSPLUGIN_API MatrixCovariance<double>;
#endif
#ifndef SOFA_DOUBLE
#endif

} // namespace container

} // namespace component

} // namespace sofa

