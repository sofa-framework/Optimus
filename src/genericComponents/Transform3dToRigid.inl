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
//    std::cout<< "TOTO" <<std::endl;
    int m = m_container->getNbPoints();
    for  (int i=0; i<m-1; i++) {

           Rigid R;

           Vector3 P0(m_container->getPX(i),m_container->getPY(i),m_container->getPZ(i));
           Vector3 P1(m_container->getPX(i+1),m_container->getPY(i+1),m_container->getPZ(i+1));

           Vector3 X = P1-P0;
           X.normalize();
           Vector3 Y,Z;

           if (fabs(dot(X,Vector3(1,0,0))) >= 0.99) {
               Y = cross(X,Vector3(0,1,0));
           }else {
               Y = cross(X,Vector3(1,0,0));
           }
           Y.normalize();
           Z = cross(X,Y);
           Z.normalize();

           Quat q;
           defaulttype::Matrix3 M(X,Y,Z);
           //defaulttype::Matrix3 N = M;
           M.transpose();
           q.fromMatrix(M);
//           q.fromFrame(X,Y,Z);
           //defaulttype::Matrix3 T= M*N;

//           std::cout << "T=" << T << std::endl;


           R.getCenter()=P0;
           R.getOrientation()=q;
           res.push_back(R);

    }

      int i = m-1;
      Rigid R;

      Vector3 pos(m_container->getPX(i),m_container->getPY(i),m_container->getPZ(i));
      Vector3 pos_prev(m_container->getPX(i-1),m_container->getPY(i-1),m_container->getPZ(i-1));

      R.getCenter()=pos;

      Vector3 X = pos-pos_prev;
      X.normalize();
      Vector3 Y,Z;


      if (fabs(dot(X,Vector3(1,0,0))) >= 0.99) {
          Y = cross(X,Vector3(0,1,0));
      }else {
          Y = cross(X,Vector3(1,0,0));
      }
      Z = cross(X,Y);

      Quat q;
      q.fromFrame(X,Y,Z);


      R.getOrientation()=q;
      res.push_back(R);


    d_out_pos.setValue(res);
}


} //constraintset

} //component

}//Sofa

#endif
