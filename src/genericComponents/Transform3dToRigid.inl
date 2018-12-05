#ifndef SOFA_COMPONENT_CONTROLLER_Transform3dToRigid_INL
#define SOFA_COMPONENT_CONTROLLER_Transform3dToRigid_INL

#include "Transform3dToRigid.h"
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/Quater.h>
#include <sofa/core/ObjectFactory.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <sofa/helper/AdvancedTimer.h>
#include <iomanip>      // std::setprecision
namespace sofa
{

namespace core
{

namespace behavior
{

using namespace core::behavior;
using namespace sofa::defaulttype;


Transform3dToRigid::Transform3dToRigid()

    : d_out_pos(initData(&d_out_pos, "out_position", "out position"))
{
    this->f_listening.setValue(true);
}

void Transform3dToRigid::init() {

    this->getContext()->get(m_container);
    if (m_container == NULL) {
        serr << " Error cannot find the edgeContainer" << sendl;
        return;
    }


    helper::vector<Rigid> res;
    int m = m_container->getNbPoints();
    {
        Rigid R;

        Vector3 P0(m_container->getPX(0),m_container->getPY(0),m_container->getPZ(0));
        Vector3 P1(m_container->getPX(1),m_container->getPY(1),m_container->getPZ(1));

        Vector3 X = P1-P0;
        X.normalize();
        Vector3 Y,Z;

        if (fabs(dot(X,Vector3(1,0,0))) >= 0.999999999999999) {
            Y = cross(X,Vector3(0,1,0));
        }else {
            Y = cross(X,Vector3(1,0,0));
        }
        Y.normalize();
        Z = cross(X,Y);
        Z.normalize();

        R.getCenter()=P0;
        R.getOrientation().fromFrame(X,Y,Z);

        res.push_back(R);
    }

    for  (int i=1; i<m-1; i++) {

        Rigid R;

        Vector3 P0(m_container->getPX(i),m_container->getPY(i),m_container->getPZ(i));
        Vector3 P1(m_container->getPX(i+1),m_container->getPY(i+1),m_container->getPZ(i+1));

        Vector3 X = P1-P0;
        X.normalize();

        Vector3 Yprec = res[i-1].getOrientation().rotate(Vector3(0,1,0));

        Vector3 Z = cross(X,Yprec);
        Z.normalize();

        Vector3 Y = cross(Z,X);
        Y.normalize();

        R.getCenter()=P0;
        R.getOrientation().fromFrame(X,Y,Z);

        res.push_back(R);

    }

    int i = m-1;{
    Rigid R;


    Vector3 pos(m_container->getPX(i),m_container->getPY(i),m_container->getPZ(i));
    Vector3 pos_prev(m_container->getPX(i-1),m_container->getPY(i-1),m_container->getPZ(i-1));


    Vector3 X = pos-pos_prev;
    X.normalize();
    Vector3 Yprec = res[i-1].getOrientation().rotate(Vector3(0,1,0));

    Vector3 Z = cross(X,Yprec);
    Z.normalize();

    Vector3 Y = cross(Z,X);
    Y.normalize();

    R.getCenter()=pos;
    R.getOrientation().fromFrame(X,Y,Z);

    res.push_back(R);

    }
    d_out_pos.setValue(res);
    std::cout << std::setprecision(std::numeric_limits<long double>::digits10 + 1)<<d_out_pos.getValue()<< std::endl;
}


} //constraintset

} //component

}//Sofa

#endif
