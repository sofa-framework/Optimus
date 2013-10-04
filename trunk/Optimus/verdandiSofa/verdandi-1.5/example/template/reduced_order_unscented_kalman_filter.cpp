#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#if defined(VERDANDI_WITH_MPI)
#include <mpi.h>
#endif

#if defined(VERDANDI_WITH_OMP)
#include <omp.h>
#endif

#include "Verdandi.hxx"
#include "seldon/SeldonSolver.hxx"

#include "model/ModelTemplate.cxx"
#include "observation_manager/ObservationManagerTemplate.cxx"
#include "method/ReducedOrderUnscentedKalmanFilter.cxx"


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

    Verdandi::ReducedOrderUnscentedKalmanFilter<
        Verdandi::ModelTemplate,
        Verdandi::ObservationManagerTemplate> driver;

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
