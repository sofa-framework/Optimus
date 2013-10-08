
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/common/common.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/AdvancedTimer.h>

/*#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK

#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#define VERDANDI_WITH_DIRECT_SOLVER


//#define SELDON_WITH_MUMPS*/

//

//#define VERDANDI_IGNORE_MESSAGE


#include "SofaModelWrapper.h"
#include "Verdandi.hxx"
#include "method/ForwardDriver.cxx"
#include "seldon/SeldonSolver.hxx"

#include "VerdandiHeader.hxx"

namespace Verdandi {
    template class Verdandi::ForwardDriver<sofa::simulation::SofaModelWrapper<double> >;
}


