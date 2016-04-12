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
#ifndef ROUKFILTER_INL
#define ROUKFILTER_INL

#include "ROUKFilter.h"


namespace sofa
{
namespace component
{
namespace stochastic
{

template <class FilterType>
ROUKFilter<FilterType>::ROUKFilter()
{    
}

template <class FilterType>
ROUKFilter<FilterType>::~ROUKFilter()
{
}

template <class FilterType>
void ROUKFilter<FilterType>::computePrediction()
{
    PRNS("computing prediction");
}


template <class FilterType>
void ROUKFilter<FilterType>::computeCorrection()
{
    PRNS("computing correction");
}

template <class FilterType>
void ROUKFilter<FilterType>::bwdInit() {
    PRNS("bwdInit");

    /// get observations TODO
    EMatrixX temp = stateWrapper->getStateErrorVarianceReduced();
    matU = temp;

    PRNW("size: " << matU.rows() << " X " << matU.cols());
    std::cout << matU << std::endl;
}


template <class FilterType>
void ROUKFilter<FilterType>::init() {
    Inherit::init();
    gnode->get(stateWrapper, core::objectmodel::BaseContext::SearchDown);

    if (stateWrapper) {
        PRNS("found stochastic state wrapper: " << stateWrapper->getName());
    } else
        PRNE("no state wrapper found!");
}

//template <class FilterType>
//void ROUKFilter<FilterType>::init()
//{
//
//}
//
//template <class FilterType>
//void ROUKFilter<FilterType>::reinit()
//{
//}

} // stochastic
} // component
} // sofa

#endif // ROUKFILTER_INL
