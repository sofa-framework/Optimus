#define VERDANDI_DEBUG_LEVEL_4

#define SELDON_WITH_ABORT
#define VERDANDI_WITH_ABORT

#include "Verdandi.hxx"
using namespace Verdandi;

#include "HamiltonJacobiBellman.cxx"

#include "QuadraticModel.cxx"

#include "LinearObservationManager.cxx"

int main(int argc, char** argv)
{

    VERDANDI_TRY;

    if (argc != 2)
    {
        string mesg  = "Usage:\n";
        mesg += string("  ") + argv[0] + " [configuration file]";
        cout << mesg << endl;
        return 1;
    }

    HamiltonJacobiBellman<double, QuadraticModel<double>,
        LinearObservationManager<double> > driver;

    driver.Initialize(argv[1]);

    while (!driver.HasFinished())
    {
        driver.InitializeStep();
        driver.Forward();
    }

    VERDANDI_END;

    return 0;

}
