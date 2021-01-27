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

#include <sofa/core/ObjectFactory.h>
#include "UKFilterClassicWithSVD.inl"
//#include <sofa/helper/accessor.h>


namespace sofa
{

namespace component
{

namespace stochastic
{


using namespace defaulttype;



SOFA_DECL_CLASS(UKFilterClassicWithSVD)

// Register in the Factory
int UKFilterClassicWithSVDClass = core::RegisterObject("UKFilterClassic")
        #ifndef SOFA_FLOAT
        .add< UKFilterClassicWithSVD<double> >()
        #endif
        #ifndef SOFA_DOUBLE
        //.add< UKFilterClassicWithSVD<float> >()
        #endif
        ;

#ifndef SOFA_FLOAT
template class SOFA_STOCHASTIC_API UKFilterClassicWithSVD<double>;
#endif
#ifndef SOFA_DOUBLE
//template class SOFA_STOCHASTIC_API UKFilterClassicWithSVD<float>;
#endif


} // namespace stochastic

} // namespace component

} // namespace sofa

