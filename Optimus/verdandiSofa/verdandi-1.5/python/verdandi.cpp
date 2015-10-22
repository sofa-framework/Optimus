// Copyright (C) 2008-2010, INRIA
// Author(s): Vivien Mallet, Claire Mouton, Marc Fragu
//
// This file is part of the data assimilation library Verdandi.
//
// Verdandi is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1 of the License, or (at your option)
// any later version.
//
// Verdandi is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
// more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Verdandi. If not, see http://www.gnu.org/licenses/.
//
// For more information, visit the Verdandi web site:
//      http://verdandi.gforge.inria.fr/


#ifndef VERDANDI_FILE_PYTHON_VERDANDI_CPP


#include "verdandi.def"

#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_DENSE

#include "Verdandi.hxx"
#include "VerdandiBase.cxx"

#include "share/VerdandiOps.cxx"

#include "seldon/SeldonSolver.hxx"

#include "model/QuadraticModel.cxx"
#include "model/ClampedBar.cxx"
#include "model/PythonModel.cxx"
#include "observation_manager/GridToNetworkObservationManager.cxx"
#include "observation_manager/LinearObservationManager.cxx"
#include "observation_manager/PythonObservationManager.cxx"
#include "method/OptimalInterpolation.cxx"
#include "method/ForwardDriver.cxx"
#include "method/ReducedOrderExtendedKalmanFilter.cxx"
#include "share/OutputSaver.cxx"

namespace Verdandi
{


    template class VSWIG_MODEL;
    template class VSWIG_MODEL1;
    class VSWIG_MODEL2;
    template class VSWIG_GRID_TO_NETWORK_OBSERVATION;
    template class VSWIG_LINEAR_OBSERVATION;
    class VSWIG_LINEAR_OBSERVATION2;
    template class VSWIG_METHOD;
    template class VSWIG_METHOD1;
    template class VSWIG_METHOD2;
    template class VSWIG_METHOD3;
    template class VSWIG_METHOD4;
    template class VSWIG_METHOD5;


}


/////////////////
// VERDANDIOPS //
/////////////////


// The following part is for the instantiation of VerdandiOps methods.

#define VERDANDI_OPS_INSTANTIATE_ELEMENT(type)                          \
    template type Verdandi::VerdandiOps::Get(string);                   \
    template type Verdandi::VerdandiOps::Get(string, string);           \
    template type Verdandi::VerdandiOps::Get(string, string, const type&); \
    template type Ops::Ops::Apply(string name, const type& arg0);       \
    template type Ops::Ops::Apply(string name, const type& arg0,        \
                                  const type& arg1);                    \
    template type Ops::Ops::Apply(string name, const type& arg0,        \
                                  const type& arg1, const type& arg2);  \
    template type Ops::Ops::Apply(string name, const type& arg0,        \
                                  const type& arg1, const type& arg2,   \
                                  const type& arg3);                    \
    template type Ops::Ops::Apply(string name, const type& arg0,        \
                                  const type& arg1, const type& arg2,   \
                                  const type& arg3, const type& arg4);  \
    template bool Verdandi::VerdandiOps::Is<type >(string);             \

#define VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(type0, type1)  \
    template void Ops::Ops::Apply(string name,                  \
                                  const std::vector<type0>& in, \
                                  std::vector<type1>& out);     \

#define VERDANDI_OPS_INSTANTIATE_VECTOR(type)                           \
    template type Verdandi::VerdandiOps::Get(string);                   \
    template type Verdandi::VerdandiOps::Get(string, string);           \
    template type Verdandi::VerdandiOps::Get(string, string, const type&); \
    template bool Verdandi::VerdandiOps::Is<type >(string);             \

VERDANDI_OPS_INSTANTIATE_ELEMENT(bool);
VERDANDI_OPS_INSTANTIATE_ELEMENT(int);
VERDANDI_OPS_INSTANTIATE_ELEMENT(float);
VERDANDI_OPS_INSTANTIATE_ELEMENT(double);
VERDANDI_OPS_INSTANTIATE_ELEMENT(string);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(bool, bool);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(int, bool);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(float, bool);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(double, bool);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(string, bool);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(bool, int);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(int, int);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(float, int);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(double, int);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(string, int);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(bool, float);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(int, float);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(float, float);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(double, float);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(string, float);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(bool, double);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(int, double);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(float, double);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(double, double);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(string, double);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(bool, string);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(int, string);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(float, string);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(double, string);
VERDANDI_OPS_INSTANTIATE_CROSSED_ELEMENT(string, string);
VERDANDI_OPS_INSTANTIATE_VECTOR(std::vector<bool>);
VERDANDI_OPS_INSTANTIATE_VECTOR(std::vector<int>);
VERDANDI_OPS_INSTANTIATE_VECTOR(std::vector<float>);
VERDANDI_OPS_INSTANTIATE_VECTOR(std::vector<double>);
VERDANDI_OPS_INSTANTIATE_VECTOR(std::vector<string>);


#define VERDANDI_FILE_PYTHON_VERDANDI_CPP
#endif
