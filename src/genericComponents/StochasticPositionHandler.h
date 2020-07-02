/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_MISC_StochasticPositionHandler_H
#define SOFA_COMPONENT_MISC_StochasticPositionHandler_H

#include "initOptimusPlugin.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/simulation/Visitor.h>
#include <fstream>

namespace sofa
{

namespace component
{

namespace misc
{

/** Write State vectors to file at a given set of time instants
 * A period can be etablished at the last time instant
 * The DoFs to print can be chosen using DOFsX and DOFsV
 * Stop to write the state if the kinematic energy reach a given threshold (stopAt)
 * The energy will be measured at each period determined by keperiod
*/
class SOFA_OPTIMUSPLUGIN_API StochasticPositionHandler: public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(StochasticPositionHandler,core::objectmodel::BaseObject);

    sofa::core::objectmodel::DataFileName f_filename;
    Data < bool > f_writeX;
    Data < bool > f_writeV;
    Data < double > f_keperiod;
    Data < bool > d_groundTruth;
    Data < bool > d_observations;
    Data< helper::vector<unsigned int> > d_indices;
    Data < bool > d_finalStochPos;

protected:
    core::behavior::BaseMechanicalState* mmodel;
    std::ofstream* outfile;

    unsigned int nextTime;
    double lastTime;



    StochasticPositionHandler();

    virtual ~StochasticPositionHandler();
public:
    virtual void init() override;

    virtual void reinit() override;

    virtual void reset() override;

//    virtual void finalStochasticState(sofa::core::objectmodel::Event* event) ;
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

    double time_prec;


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg)
    {
        if (context->getMechanicalState() == NULL)
            return false;
        return BaseObject::canCreate(obj, context, arg);
    }

};

///Create StochasticPositionHandler component in the graph each time needed
class  StochasticPositionHandlerCreator: public simulation::Visitor
{
public:
    StochasticPositionHandlerCreator(const core::ExecParams* params);
    StochasticPositionHandlerCreator(const core::ExecParams* params, const std::string &n, bool _recordX, bool _recordV, bool _recordF, bool _createInMapping, int c=0);
    virtual Result processNodeTopDown( simulation::Node*  );

    void setSceneName(std::string &n) { sceneName = n; }
    void setRecordX(bool b) {recordX=b;}
    void setRecordV(bool b) {recordV=b;}
    void setRecordF(bool b) {recordF=b;}
    void setCreateInMapping(bool b) { createInMapping=b; }
    void setCounter(int c) { counterStochasticPositionHandler = c; }
    virtual const char* getClassName() const { return "StochasticPositionHandlerCreator"; }
protected:
    std::string sceneName;
    std::string extension;
    bool recordX,recordV,recordF;
    bool createInMapping;

    int counterStochasticPositionHandler; //avoid to have two same files if two mechanical objects has the same name

    void addStochasticPositionHandler(sofa::core::behavior::BaseMechanicalState*ms, simulation::Node* gnode);

};

class  StochasticPositionHandlerActivator: public simulation::Visitor
{
public:
    StochasticPositionHandlerActivator( const core::ExecParams* params, bool active) : Visitor(params), state(active) {}
    virtual Result processNodeTopDown( simulation::Node*  );

    bool getState() const { return state; }
    void setState(bool active) { state=active; }
    virtual const char* getClassName() const { return "StochasticPositionHandlerActivator"; }
protected:
    void changeStateWriter(sofa::component::misc::StochasticPositionHandler *ws);

    bool state;
};

} // namespace misc

} // namespace component

} // namespace sofa

#endif
