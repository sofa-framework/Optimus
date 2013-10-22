#ifndef SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_INL
#define SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_INL

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/common/AnimateEndEvent.h>
#include <sofa/simulation/common/Simulation.h>
#include <sofa/simulation/common/MechanicalComputeEnergyVisitor.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/Data.h>
#include <fstream>
#include <sofa/defaulttype/Vec.h>
#include <cmath>
#include <limits>

#include "VerdandiActiveStateMonitor.h"

namespace sofa
{

namespace component
{

namespace misc
{

//using namespace sofa::defaulttype;
//using namespace std;

template <class DataTypes>
VerdandiActiveStateMonitor<DataTypes>::VerdandiActiveStateMonitor()
    : Inherit()    
    , verdandiLoopLink(initLink("verdandiLoop", "link to the verdandi animation loop"))
    //, m_objectID( initData (&m_objectID, "objectID", "unknown", "name for the object being monitored") )
{
}

/*template <class DataTypes>
VerdandiActiveStateMonitor<DataTypes>::~VerdandiActiveStateMonitor()
{
    Inherit::~Inherit();
}*/


template<class DataTypes>
void VerdandiActiveStateMonitor<DataTypes>::init()
{
    Inherit::init();
    TVerdandiAnimationLoop* vloop = verdandiLoopLink.get();

    if ( vloop != NULL) {
        std::cout << this->getName() << ": associated with loop " << vloop->getName() << std::endl;
        //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;
    } else {
        std::cerr << this->getName() << ": no verdandi loop associated!" << std::endl;
        return;
    }
}



}

}

}


#endif
