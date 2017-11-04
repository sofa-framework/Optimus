#ifndef Transform3dToRigid_H
#define Transform3dToRigid_H

#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>

#include <SofaBaseTopology/QuadSetTopologyContainer.h>
#include <SofaBaseTopology/QuadSetTopologyModifier.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <sofa/core/collision/Intersection.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>

#include <sofa/core/behavior/PairInteractionConstraint.h>
#include <sofa/core/behavior/MixedInteractionConstraint.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>

#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

#include <SofaBaseTopology/PointSetTopologyContainer.h>
#include <SofaBaseTopology/PointSetTopologyModifier.h>
#include <SofaBaseTopology/EdgeSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <sofa/core/topology/TopologyChange.h>
#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseTopology/GridTopology.h>
#include <sofa/core/topology/TopologicalMapping.h>

#include <sofa/core/visual/VisualParams.h>

#include <sofa/core/ObjectFactory.h>

#include <SofaBaseTopology/QuadSetTopologyContainer.h>
#include <SofaBaseTopology/QuadSetTopologyModifier.h>
#include <map>
#include <sofa/defaulttype/SolidTypes.h>

namespace sofa {

namespace core {

namespace behavior {

class Transform3dToRigid : public core::objectmodel::BaseObject {
public :

    SOFA_CLASS(Transform3dToRigid, core::objectmodel::BaseObject );

    typedef defaulttype::RigidTypes::Coord Rigid;
    typedef helper::vector<Rigid> VecCoord;
    typedef defaulttype::Vector3 Vector3;
    typedef defaulttype::Quat Quat;
    typedef defaulttype::SolidTypes<double>::Transform Transform;


public:

    Transform3dToRigid();
    Data <helper::vector<Rigid> > d_out_pos;

    void init();

    sofa::component::topology::EdgeSetTopologyContainer* m_container;


};

} // namespace constraintset

} // namespace component

} // namespace sofa


#endif // Transform3dToRigid_H
