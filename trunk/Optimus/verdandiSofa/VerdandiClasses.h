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

#endif
