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
/*
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/common/PrintVisitor.h>
#include <sofa/simulation/common/FindByTypeVisitor.h>
#include <sofa/simulation/common/ExportGnuplotVisitor.h>
#include <sofa/simulation/common/InitVisitor.h>
#include <sofa/simulation/common/AnimateVisitor.h>
#include <sofa/simulation/common/MechanicalVisitor.h>
#include <sofa/simulation/common/CollisionVisitor.h>
#include <sofa/simulation/common/CollisionBeginEvent.h>
#include <sofa/simulation/common/CollisionEndEvent.h>
#include <sofa/simulation/common/UpdateContextVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/simulation/common/ResetVisitor.h>
#include <sofa/simulation/common/VisualVisitor.h>
#include <sofa/simulation/common/ExportOBJVisitor.h>
#include <sofa/simulation/common/WriteStateVisitor.h>
#include <sofa/simulation/common/XMLPrintVisitor.h>
#include <sofa/simulation/common/PropagateEventVisitor.h>
#include <sofa/simulation/common/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/common/AnimateBeginEvent.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/UpdateMappingEndEvent.h>
#include <sofa/simulation/common/CleanupVisitor.h>
#include <sofa/simulation/common/DeleteVisitor.h>
#include <sofa/simulation/common/UpdateBoundingBoxVisitor.h>
#include <sofa/simulation/common/xml/NodeElement.h>

#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/system/PipeProcess.h>
#include <sofa/helper/AdvancedTimer.h>

#include <sofa/core/visual/VisualParams.h>

#include <stdlib.h>
#include <math.h>
#include <algorithm>

#include <sofa/core/VecId.h>
#include <sofa/simulation/common/MechanicalOperations.h>
#include <sofa/simulation/common/SolveVisitor.h>
#include <sofa/simulation/common/VectorOperations.h>

#include <sofa/simulation/common/IntegrateBeginEvent.h>
#include <sofa/simulation/common/IntegrateEndEvent.h>

#include "SofaModelWrapperParallel.inl"

/// should be in separated file, but does not actually work because of template functions of template classes




#include "method/ForwardDriver.cxx"
#include "method/UnscentedKalmanFilter.cxx"
#include "method/ReducedOrderUnscentedKalmanFilter.cxx"
#include "observation_manager/LinearObservationManager.cxx"

#include "seldon/SeldonHeader.hxx" // #problem, multiple definition
#include "VerdandiHeader.hxx" // #problem, multiple definition


/// should be in separated file, but does not actually work because of template functions of template classes


namespace Verdandi {
    template class Verdandi::ForwardDriver<sofa::simulation::SofaModelWrapperParallel<double> >;
    template class Verdandi::UnscentedKalmanFilter<sofa::simulation::SofaModelWrapperParallel<double>, sofa::simulation::SofaLinearObservationManagerParallel<double> >;
    template class Verdandi::ReducedOrderUnscentedKalmanFilter<sofa::simulation::SofaModelWrapperParallel<double>, sofa::simulation::SofaLinearObservationManagerParallel<double> >;

}

namespace sofa
{

namespace simulation
{


SOFA_DECL_CLASS(SofaModelWrapperParallel)
int SofaModelWrapperParallelClass = core::RegisterObject("A class implementing an interface between SOFA and verdandi; parallel version.")
        #ifndef SOFA_FLOAT
        //.add< SofaModelWrapperParallel<double> >()
        #endif
        ;

SOFA_DECL_CLASS(SofaLinearObservationManagerParallel)
int SofaLinearObservationManagerParallelClass = core::RegisterObject("A class implementing the observation manager interfaced in SOFA; parallel version.")
        #ifndef SOFA_FLOAT
        .add< SofaLinearObservationManagerParallel<double> >()
        #endif
        ;

SOFA_DECL_CLASS(MappedPointsObservationManagerParallel)
int MappedPointsObservationManagerParallelClass = core::RegisterObject("A class implementing the observation manager interfaced in SOFA; parallel version.")
        #ifndef SOFA_FLOAT
        .add< MappedPointsObservationManagerParallel<Vec3dTypes, Vec3dTypes> >()
        #endif
        ;

SOFA_DECL_CLASS(ARObservationManagerParallel)
int ARObservationManagerParallelClass = core::RegisterObject("A class implementing the observation manager interfaced in SOFA; parallel version.")
        #ifndef SOFA_FLOAT
        .add< ARObservationManagerParallel<Vec3dTypes, Vec3dTypes> >()
        #endif
        ;



SOFA_DECL_CLASS(SofaReducedOrderUKFParallel)
int SofaReducedOrderUKFParallelClass = core::RegisterObject("A class implementing the observation manager interfaced in SOFA; parallel version.")
        #ifndef SOFA_FLOAT
        .add<SofaReducedOrderUKFParallel<SofaModelWrapperParallel<double>,SofaLinearObservationManagerParallel<double> > >()
        #endif
        ;


SOFA_DECL_CLASS(SofaUnscentedKalmanFilterParallel)
int SofaUnscentedKalmanFilterParallelClass = core::RegisterObject("A class implementing the observation manager interfaced in SOFA; parallel version.")
        #ifndef SOFA_FLOAT
        .add<SofaUnscentedKalmanFilterParallel<SofaModelWrapperParallel<double>,SofaLinearObservationManagerParallel<double> > >()
        #endif
        ;


SOFA_DECL_CLASS(SofaForwardDriverParallel)
int SofaForwardDriverParallelClass = core::RegisterObject("A class implementing the observation manager interfaced in SOFA; parallel version.")
        #ifndef SOFA_FLOAT
        .add<SofaForwardDriverParallel<SofaModelWrapperParallel<double> > >()
        #endif
        ;


template class SofaModelWrapperParallel<double>;

/// observation managers:
// #problem, even these cause segfault..
template class SofaLinearObservationManagerParallel<double>;
template class MappedPointsObservationManagerParallel<sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes>;
template class ARObservationManagerParallel<sofa::defaulttype::Vec3dTypes, sofa::defaulttype::Vec3dTypes>;


/// filters:
template class SofaForwardDriverParallel<SofaModelWrapperParallel<double> >;
template class SofaReducedOrderUKFParallel<SofaModelWrapperParallel<double>,SofaLinearObservationManagerParallel<double> >;
template class SofaUnscentedKalmanFilterParallel<SofaModelWrapperParallel<double>,SofaLinearObservationManagerParallel<double> >;



} // namespace simulation

} // namespace sofa*/
