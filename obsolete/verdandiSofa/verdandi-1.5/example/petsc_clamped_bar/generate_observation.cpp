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
#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#define VERDANDI_WITH_PETSC

//#define VERDANDI_WITH_DIRECT_SOLVER
//#define SELDON_WITH_MUMPS

#define VERDANDI_WITH_MPI

#if defined(VERDANDI_WITH_MPI)
#include <mpi.h>
#endif

#include <petscmat.h>
#include <petscvec.h>
#include <petscksp.h>

#include "Verdandi.hxx"
#include "seldon/SeldonSolver.hxx"

#include "seldon/vector/PetscVector.cxx"
#include "seldon/matrix/PetscMatrix.cxx"

#include "model/PetscClampedBar.cxx"
#include "method/ObservationGenerator.cxx"
#include "observation_manager/PetscLinearObservationManager.cxx"

static char help[] = "Observation Generator.\n\n";

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

    PetscInitialize(&argc, &argv, (char *)0, help);

    Verdandi::ObservationGenerator<Verdandi::PetscClampedBar<double> ,
        Verdandi::PetscLinearObservationManager<double> >
        driver;

    driver.Initialize(argv[1]);

    while (!driver.HasFinished())
    {
        driver.InitializeStep();
        driver.Forward();
    }

    driver.Finalize();

    VERDANDI_END;

    int ierr;
    ierr = PetscFinalize();
    CHKERRQ(ierr);

    return 0;

}
