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
// You may use 'LinearObservationManager' or 'GridToNetworkObservationManager'
// as observation operator.
#ifndef OBSERVATION_OPERATOR
#define OBSERVATION_OPERATOR LinearObservationManager
#endif


#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_DENSE

#include "Verdandi.hxx"

#include "seldon/SeldonSolver.hxx"

#include "model/ShallowWater.cxx"
#include "method/OptimalInterpolation.cxx"

#define _QUOTE(x) #x
#define QUOTE(x) _QUOTE(x)
#include QUOTE(OBSERVATION_OPERATOR.cxx)


int main(int argc, char** argv)
{

    VERDANDI_TRY;

    if (argc != 2)
    {
        string mesg  = "Usage:\n";
        mesg += string("  ") + argv[0] + " [configuration file]";
        std::cout << mesg << std::endl;
        return 1;
    }

    typedef double real;

    Verdandi::OptimalInterpolation<Verdandi::ShallowWater<real>,
        Verdandi::OBSERVATION_OPERATOR<real> > driver;

    driver.Initialize(argv[1]);

    while (!driver.HasFinished())
    {
        driver.InitializeStep();
        driver.Forward();
        driver.Analyze();
        driver.FinalizeStep();
    }

    driver.Finalize();

    VERDANDI_END;

    return 0;

}
