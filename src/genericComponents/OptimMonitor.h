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
#ifndef SOFA_COMPONENT_MISC_OPTIMMONITOR_H
#define SOFA_COMPONENT_MISC_OPTIMMONITOR_H
#include "initOptimusPlugin.h"

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/types/RGBAColor.h>
#include <sofa/core/objectmodel/DataFileName.h>

namespace sofa
{

namespace component
{

namespace misc
{

/**
 * Class which replaces the default monitor in SOFA. Differences:
 * 1. Export on special Optimus events (see initialization according to d_exportOnEvent
 * 2. Reinitialize pointers to mechanical object data everytime before the export (needed for some Python scenes)
 **/

template <class DataTypes>
class OptimMonitor: public virtual core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(OptimMonitor, DataTypes), core::objectmodel::BaseObject);

    typedef sofa::helper::types::RGBAColor RGBAColor;
    typedef typename DataTypes::VecReal VecReal;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;

protected:
    OptimMonitor ();
    ~OptimMonitor ();
public:
    /// init data
    virtual void init () override;

    /// reset OptimMonitored values
    virtual void reset () override;

    /** initialize gnuplot files
        * called when ExportGnuplot box is checked
    */
    virtual void reinit() override;

    /** function called at every step of simulation;
        * store mechanical state vectors (forces, positions, velocities) into
        * the OptimMonitorData nested class. The filter (which position(s), velocity(ies) or *force(s) are displayed) is made in the gui
    */
    virtual void handleEvent( core::objectmodel::Event* ev ) override;

    virtual void draw (const core::visual::VisualParams* vparams) override;

    /// create gnuplot files
    virtual void initGnuplot ( const std::string path );

    /// write in gnuplot files the OptimMonitored desired data (velocities,positions,forces)
    virtual void exportGnuplot ( Real time );

    virtual std::string getTemplateName() const override
    {
        return templateName(this);
    }

    static std::string templateName(const OptimMonitor<DataTypes>* = NULL)
    {
        return DataTypes::Name();
    }

    /// Editable Data
    Data< helper::vector<unsigned int> > d_indices;

    Data< bool > d_saveXToGnuplot; ///< export OptimMonitored positions as gnuplot file
    Data< bool > d_saveVToGnuplot; ///< export OptimMonitored velocities as gnuplot file
    Data< bool > d_saveFToGnuplot; ///< export OptimMonitored forces as gnuplot file

    Data< bool > d_showPositions; ///< see the OptimMonitored positions
    Data<RGBAColor > d_positionsColor; ///< define the color of positions

    Data< bool > d_showVelocities; ///< see the OptimMonitored velocities
    Data< RGBAColor > d_velocitiesColor; ///< define the color of velocities

    Data< bool > d_showForces; ///< see the OptimMonitored forces
    Data< RGBAColor > d_forcesColor; ///< define the color of forces

    Data< double > d_showMinThreshold; ///< under this value, vectors are not represented

    Data< bool > d_showTrajectories; ///< print the trajectory of OptimMonitored particles
    Data< double > d_trajectoriesPrecision; ///< set the dt between to save of positions
    Data< RGBAColor > d_trajectoriesColor; ///< define the color of the trajectories

    Data< double > d_showSizeFactor; ///< factor to multiply to arrows
    core::objectmodel::DataFileName  d_fileName;
    Data< bool > d_saveZeroStep;
    Data< int > d_exportOnEvent;
    Data< double > d_timeShift;

protected:

    std::ofstream* m_saveGnuplotX;
    std::ofstream* m_saveGnuplotV;
    std::ofstream* m_saveGnuplotF;

    const VecCoord * m_X; ///< positions of the mechanical object monitored;
    const VecDeriv * m_V; ///< velocities of the mechanical object monitored;
    const VecDeriv * m_F; ///< forces of the mechanical object monitored;


    double m_saveDt; ///< use for trajectoriesPrecision (save to file value only if trajectoriesPrecision <= internalDt)
    double m_internalDt; ///< use for trajectoriesPrecision (save value only if trajectoriesPrecision <= internalDt)

    sofa::helper::vector < sofa::helper::vector<Coord> > m_savedPos; ///< store all the monitored positions, for trajectories display
};

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(SOFA_COMPONENT_MISC_OPTIMMONITOR_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<defaulttype::Vec3dTypes>;
extern template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<defaulttype::Vec6dTypes>;
extern template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<defaulttype::Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<defaulttype::Vec3fTypes>;
extern template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<defaulttype::Vec6fTypes>;
extern template class SOFA_OPTIMUSPLUGIN_API OptimMonitor<defaulttype::Rigid3fTypes>;
#endif
#endif


} // namespace misc

} // namespace component

} // namespace sofa

#endif
