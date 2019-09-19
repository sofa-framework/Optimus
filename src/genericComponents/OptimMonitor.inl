/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
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
#ifndef SOFA_COMPONENT_MISC_OPTIMMONITOR_INL
#define SOFA_COMPONENT_MISC_OPTIMMONITOR_INL

#include "OptimMonitor.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/Simulation.h>
#include <sofa/defaulttype/DataTypeInfo.h>
#include <sofa/core/objectmodel/Context.h>
#include <sofa/core/objectmodel/Data.h>
#include <fstream>
#include <iomanip>
#include <sofa/defaulttype/Vec.h>
#include <cmath>

#include "../genericComponents/FilterEvents.h"




namespace sofa
{

namespace component
{

namespace misc
{


///////////////////////////// OptimMonitor /////////////////////////////////////
template <class DataTypes>
OptimMonitor<DataTypes>::OptimMonitor()
    : d_indices ( initData ( &d_indices, "indices", "MechanicalObject points indices to monitor" ) )
    , d_saveXToGnuplot ( initData ( &d_saveXToGnuplot, false, "ExportPositions", "export OptimMonitored positions as gnuplot file" ) )
    , d_saveVToGnuplot ( initData ( &d_saveVToGnuplot, false, "ExportVelocities", "export OptimMonitored velocities as gnuplot file" ) )
    , d_saveFToGnuplot ( initData ( &d_saveFToGnuplot, false, "ExportForces", "export OptimMonitored forces as gnuplot file" ) )
    , d_showPositions (initData (&d_showPositions, false, "showPositions", "see the OptimMonitored positions"))
    , d_positionsColor (initData (&d_positionsColor, "PositionsColor", "define the color of positions"))
    , d_showVelocities (initData (&d_showVelocities, false, "showVelocities", "see the OptimMonitored velocities"))
    , d_velocitiesColor(initData (&d_velocitiesColor, "VelocitiesColor", "define the color of velocities"))
    , d_showForces (initData (&d_showForces, false, "showForces", "see the OptimMonitored forces"))
    , d_forcesColor (initData (&d_forcesColor, "ForcesColor", "define the color of forces"))
    , d_showMinThreshold (initData (&d_showMinThreshold, 0.01 ,"showMinThreshold", "under this value, vectors are not represented"))
    , d_showTrajectories (initData (&d_showTrajectories, false ,"showTrajectories", "print the trajectory of OptimMonitored particles"))
    , d_trajectoriesPrecision (initData (&d_trajectoriesPrecision, 0.001,"TrajectoriesPrecision", "set the dt between to save of positions"))
    , d_trajectoriesColor(initData (&d_trajectoriesColor, "TrajectoriesColor", "define the color of the trajectories"))
    , d_showSizeFactor(initData (&d_showSizeFactor, 1.0, "sizeFactor", "factor to multiply to arrows"))
    , d_fileName(initData (&d_fileName, "fileName", "name of the plot files to be generated"))
    , d_saveZeroStep(initData (&d_saveZeroStep, true, "saveZeroStep", "save positions for zero time step"))
    , d_exportOnEvent (initData (&d_exportOnEvent, 0, "exportOnEvent", "on which event data are exported: -1: no export, 0: animate end event, 1: after prediction, 2: after correction"))
    , d_timeShift (initData (&d_timeShift, 0.0, "timeShift", "add shift to time when exporting data"))
    , m_saveGnuplotX ( NULL ), m_saveGnuplotV ( NULL ), m_saveGnuplotF ( NULL )
    , m_X (NULL), m_V(NULL), m_F(NULL)
    , m_saveDt(0.0)
    , m_internalDt(0.0)

{
    if (!f_listening.isSet()) f_listening.setValue(true);

    d_positionsColor=RGBAColor::yellow();
    d_velocitiesColor=RGBAColor::yellow();
    d_forcesColor=RGBAColor::yellow();
    d_trajectoriesColor=RGBAColor::yellow();
}
/////////////////////////// end OptimMonitor ///////////////////////////////////



////////////////////////////// ~OptimMonitor ///////////////////////////////////
template <class DataTypes>
OptimMonitor<DataTypes>::~OptimMonitor()
{
    if (m_saveGnuplotX) delete ( m_saveGnuplotX );
    if (m_saveGnuplotV) delete ( m_saveGnuplotV );
    if (m_saveGnuplotF) delete ( m_saveGnuplotF );
}
///////////////////////////// end~OptimMonitor /////////////////////////////////




////////////////////////////// init () ////////////////////////////////////
template<class DataTypes>
void OptimMonitor<DataTypes>::init()
{
    if (!d_fileName.isSet()) {
        d_fileName.setValue(std::string("./") + getName());
    }

    core::behavior::MechanicalState<DataTypes>* mmodel = dynamic_cast<core::behavior::MechanicalState<DataTypes>*>( this->getContext()->getMechanicalState() );

    if(!mmodel)
    {
        msg_error("OptimMonitor") << "error : no MechanicalObject found";
        return;
    }

    m_X = &mmodel->read(core::ConstVecCoordId::position())->getValue();
    m_V = &mmodel->read(core::ConstVecDerivId::velocity())->getValue();
    m_F = &mmodel->read(core::ConstVecDerivId::force())->getValue();

    initGnuplot (d_fileName.getFullPath());

    m_savedPos.resize(d_indices.getValue().size());

    ///  initial export of the data
    if (d_saveZeroStep.getValue()) {
        if ( d_saveXToGnuplot.getValue() || d_saveVToGnuplot.getValue() || d_saveFToGnuplot.getValue() ) {
            m_saveDt += this->getContext()->getDt();

            if (d_trajectoriesPrecision.getValue() <= m_saveDt)
            {
                m_saveDt = 0.0;
                exportGnuplot ( (Real) this ->getTime() );
            }
        }

        if (d_showTrajectories.getValue())
        {
            m_internalDt += this->getContext()->getDt();

            if (d_trajectoriesPrecision.getValue() <= m_internalDt)
            {
                m_internalDt = 0.0;
                for (unsigned int i=0; i < d_indices.getValue().size(); ++i)
                {
                    m_savedPos[i].push_back( (*m_X)[d_indices.getValue()[i]] );
                }
            }
        }
    }
}
///////////////////////////// end init () /////////////////////////////////



///////////////////////////// reset () ////////////////////////////////////
template<class DataTypes>
void OptimMonitor<DataTypes>::reset()
{
    m_internalDt = 0.0;
    for(unsigned int i=0 ; i<d_indices.getValue().size() ; ++i)
        m_savedPos[i].clear();
}
//////////////////////////// end reset () /////////////////////////////////



//////////////////////////// reinit () ////////////////////////////////////
template<class DataTypes>
void OptimMonitor<DataTypes>::reinit()
{
    init();
}
/////////////////////////// end reinit () /////////////////////////////////



template<class DataTypes>
void OptimMonitor<DataTypes>::handleEvent( core::objectmodel::Event* ev )
{

    bool exportEvent = false;
    int exportOnEvent = d_exportOnEvent.getValue();

    switch (exportOnEvent) {
    case -1: break;
    case 0: exportEvent = sofa::simulation::AnimateEndEvent::checkEventType(ev); break;
    case 1: exportEvent = sofa::component::stochastic::PredictionEndEvent::checkEventType(ev); break;
    case 2: exportEvent = sofa::component::stochastic::CorrectionEndEvent::checkEventType(ev); break;
    }

    if (exportEvent)
        PRNS("Exporting on event " << exportOnEvent);

    if (exportEvent)
    {
        core::behavior::MechanicalState<DataTypes>* mmodel = dynamic_cast<core::behavior::MechanicalState<DataTypes>*>( this->getContext()->getMechanicalState() );

        if(!mmodel)
        {
            msg_error("OptimMonitor") << "error : no MechanicalObject found";
            return;
        }

        /// reinit the pointers (as they change in some Python scenes
        m_X = &mmodel->read(core::ConstVecCoordId::position())->getValue();
        m_V = &mmodel->read(core::ConstVecDerivId::velocity())->getValue();
        m_F = &mmodel->read(core::ConstVecDerivId::force())->getValue();

        if ( d_saveXToGnuplot.getValue() || d_saveVToGnuplot.getValue() || d_saveFToGnuplot.getValue() ) {
            m_saveDt += this->getContext()->getDt();

            if (d_trajectoriesPrecision.getValue() <= m_saveDt)
            {
                m_saveDt = 0.0;
                exportGnuplot ( (Real) this ->getTime() );
            }
        }

        if (d_showTrajectories.getValue())
        {
            m_internalDt += this->getContext()->getDt();

            if (d_trajectoriesPrecision.getValue() <= m_internalDt)
            {
                m_internalDt = 0.0;
                for (unsigned int i=0; i < d_indices.getValue().size(); ++i)
                {
                    m_savedPos[i].push_back( (*m_X)[d_indices.getValue()[i]] );
                }
            }
        }
    }
}

/////////////////////////// draw () ////////////////////////////////////
template<class DataTypes>
void OptimMonitor<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    vparams->drawTool()->setLightingEnabled(false);
    if (d_showPositions.getValue())
    {
        helper::vector<defaulttype::Vector3> points;
        for (unsigned int i=0; i < d_indices.getValue().size(); ++i)
        {
            Coord posvertex = (*m_X)[d_indices.getValue()[i]];
            points.push_back(defaulttype::Vector3(posvertex[0],posvertex[1],posvertex[2]));
        }
        vparams->drawTool()->drawPoints(points, (float)(d_showSizeFactor.getValue())*2.0f, d_positionsColor.getValue());

    }

    if (d_showVelocities.getValue())
    {
        for (unsigned int i=0; i < d_indices.getValue().size(); ++i)
        {
            Coord posVertex = (*m_X)[d_indices.getValue()[i]];
            defaulttype::Vector3 p1(posVertex[0],posVertex[1],posVertex[2]);
            Deriv velVertex = (*m_V)[d_indices.getValue()[i]];
            defaulttype::Vector3 p2(d_showSizeFactor.getValue()*velVertex[0],d_showSizeFactor.getValue()*velVertex[1],d_showSizeFactor.getValue()*velVertex[2]);

            if(p2.norm() > d_showMinThreshold.getValue())
                vparams->drawTool()->drawArrow(p1, p1+p2, (float)(d_showSizeFactor.getValue()*p2.norm()/20.0), d_velocitiesColor.getValue());
        }
    }

    if (d_showForces.getValue() && m_F->size()>0)
    {
        for (unsigned int i=0; i < d_indices.getValue().size(); ++i)
        {
            Coord posVertex = (*m_X)[d_indices.getValue()[i]];
            defaulttype::Vector3 p1(posVertex[0],posVertex[1],posVertex[2]);
            Deriv forceVertex = (*m_F)[d_indices.getValue()[i]];
            defaulttype::Vector3 p2(d_showSizeFactor.getValue()*forceVertex[0],d_showSizeFactor.getValue()*forceVertex[1],d_showSizeFactor.getValue()*forceVertex[2]);

            if(p2.norm() > d_showMinThreshold.getValue())
                vparams->drawTool()->drawArrow(p1, p1+p2, (float)(d_showSizeFactor.getValue()*p2.norm()/20.0), d_forcesColor.getValue());
        }
    }

    if (d_showTrajectories.getValue())
    {
        m_internalDt += this->getContext()->getDt();
        for (unsigned int i=0; i < d_indices.getValue().size(); ++i)
        {
            helper::vector<defaulttype::Vector3> points;
            Coord point;
            for (unsigned int j=0 ; j<m_savedPos[i].size() ; ++j)
            {
                point = m_savedPos[i][j];
                points.push_back(defaulttype::Vector3(point[0], point[1], point[2]));
                if(j!=0)
                    points.push_back(defaulttype::Vector3(point[0], point[1], point[2]));
            }
            vparams->drawTool()->drawLines(points, (float)(d_showSizeFactor.getValue()*0.2), d_trajectoriesColor.getValue());
        }
    }
}
/////////////////////////// end draw () ////////////////////////////////







/////////////////////////// initGnuplot () ////////////////////////////////
template<class DataTypes>
void OptimMonitor<DataTypes>::initGnuplot ( const std::string path )
{
    if ( !this->getName().empty() )
    {
        if ( d_saveXToGnuplot.getValue() )
        {
            if ( m_saveGnuplotX != NULL ) delete m_saveGnuplotX;
            m_saveGnuplotX = new std::ofstream ( ( path + "_x.txt" ).c_str() );
            ( *m_saveGnuplotX ) << "# Gnuplot File : positions of "
                                << d_indices.getValue().size() << " particle(s) OptimMonitored"
                                <<  std::endl;
            ( *m_saveGnuplotX ) << "# 1st Column : time, others : particle(s) number ";

            for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
                ( *m_saveGnuplotX ) << d_indices.getValue()[i] << " ";
            ( *m_saveGnuplotX ) << std::endl;

        }

        if ( d_saveVToGnuplot.getValue() )
        {
            if ( m_saveGnuplotV != NULL ) delete m_saveGnuplotV;

            m_saveGnuplotV = new std::ofstream ( ( path + "_v.txt" ).c_str() );
            ( *m_saveGnuplotV ) << "# Gnuplot File : velocities of "
                                << d_indices.getValue().size() << " particle(s) OptimMonitored"
                                <<  std::endl;
            ( *m_saveGnuplotV ) << "# 1st Column : time, others : particle(s) number ";

            for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
                ( *m_saveGnuplotV ) << d_indices.getValue()[i] << " ";
            ( *m_saveGnuplotV ) << std::endl;
        }



        if ( d_saveFToGnuplot.getValue() )
        {
            if ( m_saveGnuplotF != NULL ) delete m_saveGnuplotF;
            m_saveGnuplotF = new std::ofstream ( ( path + "_f.txt" ).c_str() );
            ( *m_saveGnuplotF ) << "# Gnuplot File : forces of "
                                << d_indices.getValue().size() << " particle(s) OptimMonitored"
                                <<  std::endl;
            ( *m_saveGnuplotF ) << "# 1st Column : time, others : particle(s) number ";

            for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
                ( *m_saveGnuplotF ) << d_indices.getValue()[i] << " ";
            ( *m_saveGnuplotF ) << std::endl;
        }

    }
}
////////////////////////// end initGnuplot () /////////////////////////////



///////////////////////// exportGnuplot () ////////////////////////////////
template<class DataTypes>
void OptimMonitor<DataTypes>::exportGnuplot ( Real time )
{
    if ( d_saveXToGnuplot.getValue() )
    {
        ( *m_saveGnuplotX ) << (time - d_timeShift.getValue() < 1e-08 ? 0.00 : time - d_timeShift.getValue()) <<"\t";

        for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
            ( *m_saveGnuplotX ) << (*m_X)[d_indices.getValue()[i]] << "\t";
        ( *m_saveGnuplotX ) << std::endl;
    }
    if ( d_saveVToGnuplot.getValue() && m_V->size()>0 )
    {
        ( *m_saveGnuplotV ) << (time - d_timeShift.getValue() < 1e-08 ? 0.00 : time - d_timeShift.getValue()) <<"\t";

        for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
            ( *m_saveGnuplotV ) << (*m_V)[d_indices.getValue()[i]] << "\t";
        ( *m_saveGnuplotV ) << std::endl;
    }

    if ( d_saveFToGnuplot.getValue() && m_F->size()>0)
    {
        ( *m_saveGnuplotF ) << (time - d_timeShift.getValue() < 1e-08 ? 0.00 : time - d_timeShift.getValue()) <<"\t";

        for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
            ( *m_saveGnuplotF ) << (*m_F)[d_indices.getValue()[i]] << "\t";
        ( *m_saveGnuplotF ) << std::endl;
    }
}
///////////////////////////////////////////////////////////////////////////

} // namespace misc

} // namespace component

} // namespace sofa

#endif
