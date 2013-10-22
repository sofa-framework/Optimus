#ifndef SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_H
#define SOFA_COMPONENT_MISC_VERDANDI_ACTIVE_STATE_MONITOR_H

#include <sofa/component/misc/Monitor.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>

#include "VerdandiAnimationLoop.h"

//#include "../SteerableNeedlesModeling/SteerableNeedleTissueParameters.h"

namespace sofa
{

namespace component
{

namespace misc
{

template<class DataTypes>
class VerdandiActiveStateMonitor : public virtual Monitor<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(VerdandiActiveStateMonitor,DataTypes), SOFA_TEMPLATE(Monitor,DataTypes));

    //typedef sofa::component::controller::SimulationPlanningController<defaulttype::Rigid3dTypes> TPlanningController;
    typedef sofa::simulation::VerdandiAnimationLoop TVerdandiAnimationLoop;

    SingleLink<VerdandiActiveStateMonitor<DataTypes>, TVerdandiAnimationLoop, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> verdandiLoopLink;
    //Data<string> m_objectID;

    typedef Monitor<DataTypes> Inherit;
    typedef typename DataTypes::VecReal VecReal;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;

    //typedef typename sofa::component::container::SteerableNeedleTissueParameters<Real> TTissueParameters;
    //typedef typename Coord::Pos Pos;
protected:
    VerdandiActiveStateMonitor();
    //~VerdandiActiveStateMonitor();*/
    //size_t controllerIndex;
public:
    virtual void init();
    virtual void bwdInit() {}
    //virtual void handleEvent(core::objectmodel::Event *event) {}


};

}

}

}

#endif
