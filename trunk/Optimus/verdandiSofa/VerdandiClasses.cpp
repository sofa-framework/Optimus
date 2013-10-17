
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>
#include <sofa/core/ExecParams.h>
#include <sofa/simulation/common/common.h>
#include <sofa/simulation/common/Node.h>
#include <sofa/helper/AdvancedTimer.h>

#define VERDANDI_DEBUG_LEVEL_4
#define SELDON_WITH_BLAS
#define SELDON_WITH_LAPACK
#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

/*#define VERDANDI_WITH_ABORT
#define VERDANDI_DENSE

#define VERDANDI_WITH_DIRECT_SOLVER
//#define SELDON_WITH_MUMPS*/

//

//#define VERDANDI_IGNORE_MESSAGE

#include "VerdandiHeader.hxx"

#include "SofaModelWrapper.h"
#include "Verdandi.hxx"
#include "seldon/Seldon.hxx"
#include "method/ForwardDriver.cxx"
#include "method/UnscentedKalmanFilter.cxx"
#include "method/ReducedOrderUnscentedKalmanFilter.cxx"
#include "seldon/SeldonSolver.hxx"
#include "observation_manager/LinearObservationManager.cxx"
#include "seldon/vector/VectorCollection.hxx"


namespace Verdandi {    
    template class Verdandi::ForwardDriver<sofa::simulation::SofaModelWrapper<double> >;
    template class Verdandi::UnscentedKalmanFilter<sofa::simulation::SofaModelWrapper<double>, Verdandi::LinearObservationManager<double> >;
    template class Verdandi::ReducedOrderUnscentedKalmanFilter<sofa::simulation::SofaModelWrapper<double>, Verdandi::LinearObservationManager<double> >;

}

/*namespace Seldon {
    template <class T, class Prop, class Storage, class Allocator> ostream& operator << (ostream& out, const Matrix<T, Prop, Storage, Allocator>& A);
}*/


