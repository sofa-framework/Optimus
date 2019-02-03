#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_DENSE
#define VERDANDI_WITH_ABORT

#if defined(VERDANDI_WITH_MPI)
#include <mpi.h>
#endif

#include "Verdandi.hxx"

#include "model/ModelTemplate.cxx"
#include "observation_manager/ObservationManagerTemplate.cxx"
#include "method/EnsembleKalmanFilter.cxx"
#include "method/TR1PerturbationManager.cxx"

int main(int argc, char** argv)
{

    TRY;

    if (argc != 2)
    {
        string mesg  = "Usage:\n";
        mesg += string("  ") + argv[0] + " [configuration file]";
        std::cout << mesg << std::endl;
        return 1;
    }

    Verdandi::EnsembleKalmanFilter<Verdandi::ModelTemplate,
        Verdandi::ObservationManagerTemplate,
        Verdandi::TR1PerturbationManager> driver;

    driver.Initialize(argv[1]);

    while (!driver.HasFinished())
    {
        driver.InitializeStep();
        driver.Forward();
        driver.Analyze();
    }

    END;

    return 0;

}
