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
#ifndef SOFA_SIMULATION_VERDANDICLASSES_H
#define SOFA_SIMULATION_VERDANDICLASSES_H

#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#define VERDANDI_WITH_DIRECT_SOLVER
//#define SELDON_WITH_MUMPS

/// generic headers:
#include "VerdandiHeader.hxx"
#include "seldon/SeldonHeader.hxx"

///actually used methods
#include "method/ForwardDriver.hxx"
#include "method/UnscentedKalmanFilter.hxx"
#include "method/ReducedOrderUnscentedKalmanFilter.hxx"

#include "observation_manager/LinearObservationManager.hxx"
#include "observation_manager/ObservationAggregator.hxx"
#include "method/SigmaPoint.hxx"
#include "seldon/computation/interfaces/Lapack_LinearEquations.hxx"
//#include "share/VerdandiOps.hxx"

#endif
