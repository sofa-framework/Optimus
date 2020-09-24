/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "VerdandiActiveStateMonitor.inl"

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>



namespace sofa
{

namespace component
{

namespace misc
{

SOFA_DECL_CLASS(VerdandiActiveStateMonitor)

using namespace sofa::defaulttype;

// Register in the Factory
int VerdandiActiveStateMonitorClass = core::RegisterObject("Monitoring of particles")
#ifndef SOFA_FLOAT
        .add< VerdandiActiveStateMonitor<Vec3dTypes> >(true)
//        .add< VerdandiActiveStateMonitor<Rigid3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< VerdandiActiveStateMonitor<Vec3fTypes> >()
//        .add< VerdandiActiveStateMonitor<Rigid3fTypes> >()
#endif
        ;



#ifndef SOFA_FLOAT
template class VerdandiActiveStateMonitor<Vec3dTypes>;
//template class VerdandiActiveStateMonitor<Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class VerdandiActiveStateMonitor<Vec3fTypes>;
//template class VerdandiActiveStateMonitor<Rigid3fTypes>;
#endif


/*template<>
void VerdandiActiveStateMonitor<Vec3dTypes>::bwdInit() {
    if (this->X != NULL) {
        //std::cout << this->getName() << ": size of the state: " << this->X->size() << std::endl;

        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            //std::cout << this->getName() << ": associated with controller " << contr->getName() << std::endl;
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(x);
            }
            contr->objectPositions.push_back(pos);
            contr->objectIDs.push_back(m_objectID.getValue());
            controllerIndex = contr->objectPositions.size() - 1;           
        } else {
            std::cerr << this->getName() << ": no controller associated" << std::endl;
            return;
        }

    } else {
        std::cerr << this->getName() << ": no position vector associated!" << std::endl;
        return;
    }
}

template<>
void VerdandiActiveStateMonitor<Vec3fTypes>::bwdInit() {
    if (this->X != NULL) {
        //std::cout << this->getName() << ": size of the state: " << this->X->size() << std::endl;

        TPlanningController* contr = planningController.get();       

        if ( contr != NULL) {
            //std::cout << this->getName() << ": associated with controller " << contr->getName() << std::endl;
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(x);
            }
            contr->objectPositions.push_back(pos);
            contr->objectIDs.push_back(m_objectID.getValue());
            controllerIndex = contr->objectPositions.size() - 1;
        } else {
            std::cerr << this->getName() << ": no controller associated" << std::endl;
            return;
        }

    } else {
        std::cerr << this->getName() << ": no position vector associated!" << std::endl;
        return;
    }
}


template<>
void VerdandiActiveStateMonitor<Rigid3dTypes>::bwdInit() {
    if (this->X != NULL) {
        //std::cout << this->getName() << ": size of the state: " << this->X->size() << std::endl;

        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            //std::cout << this->getName() << ": associated with controller " << contr->getName() << std::endl;
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(Rigid3dTypes::getCPos(x));
            }
            contr->objectPositions.push_back(pos);
            contr->objectIDs.push_back(m_objectID.getValue());
            controllerIndex = contr->objectPositions.size() - 1;
        } else {
            std::cerr << this->getName() << ": no controller associated" << std::endl;
            return;
        }

    } else {
        std::cerr << this->getName() << ": no position vector associated!" << std::endl;
        return;
    }
}


template<>
void VerdandiActiveStateMonitor<Rigid3fTypes>::bwdInit() {
    if (this->X != NULL) {
        //std::cout << this->getName() << ": size of the state: " << this->X->size() << std::endl;

        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            std::cout << this->getName() << ": associated with controller " << contr->getName() << std::endl;
            std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(Rigid3dTypes::getCPos(x));
            }
            contr->objectPositions.push_back(pos);
            contr->objectIDs.push_back(m_objectID.getValue());
            controllerIndex = contr->objectPositions.size() - 1;
        } else {
            std::cerr << this->getName() << ": no controller associated" << std::endl;
            return;
        }

    } else {
        std::cerr << this->getName() << ": no position vector associated!" << std::endl;
        return;
    }
}



template<>
void VerdandiActiveStateMonitor<Vec3dTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(x);
            }
           contr->objectPositions[controllerIndex] = pos;
        }

    }
}

template<>
void VerdandiActiveStateMonitor<Vec3fTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(x);
            }
            contr->objectPositions[controllerIndex] = pos;
        }
    }
}

template<>
void VerdandiActiveStateMonitor<Rigid3dTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(Rigid3dTypes::getCPos(x));
            }
            contr->objectPositions[controllerIndex] = pos;
        }
    }
}

template<>
void VerdandiActiveStateMonitor<Rigid3fTypes>::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        TPlanningController* contr = planningController.get();

        if ( contr != NULL) {
            //std::cout << this->getName() << ": size of object positions: " << contr->objectPositions.size() << std::endl;

            std::vector<TPlanningController::Pos> pos;
            pos.clear();

            for (size_t i = 0; i < this->X->size(); i++) {
                Coord x = this->X->at(i);
                pos.push_back(Rigid3dTypes::getCPos(x));
            }
            contr->objectPositions[controllerIndex] = pos;
        }
    }
}*/




} // namespace misc

} // namespace component

} // namespace sofa
