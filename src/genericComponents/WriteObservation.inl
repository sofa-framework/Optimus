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
#ifndef SOFA_COMPONENT_MISC_WriteObservation_INL
#define SOFA_COMPONENT_MISC_WriteObservation_INL

#include <genericComponents/WriteObservation.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/DataFileName.h>

#include <fstream>
#include <sstream>

namespace sofa
{

namespace component
{

namespace misc
{


WriteObservation::WriteObservation()
    : f_filename( initData(&f_filename, "filename", "output file name"))
    , f_writeX( initData(&f_writeX, true, "writeX", "flag enabling output of X vector"))
    , f_writeX0( initData(&f_writeX0, false, "writeX0", "flag enabling output of X0 vector"))
    , f_writeV( initData(&f_writeV, false, "writeV", "flag enabling output of V vector"))
    , f_writeF( initData(&f_writeF, false, "writeF", "flag enabling output of F vector"))
    , f_interval( initData(&f_interval, 0.0, "interval", "time duration between outputs"))
    , f_time( initData(&f_time, helper::vector<double>(0), "time", "set time to write outputs"))
    , f_period( initData(&f_period, 0.0, "period", "period between outputs"))
    , f_DOFsX( initData(&f_DOFsX, helper::vector<unsigned int>(0), "DOFsX", "set the position DOFs to write"))
    , f_DOFsV( initData(&f_DOFsV, helper::vector<unsigned int>(0), "DOFsV", "set the velocity DOFs to write"))
    , f_stopAt( initData(&f_stopAt, 0.0, "stopAt", "stop the simulation when the given threshold is reached"))
    , f_keperiod( initData(&f_keperiod, 0.0, "keperiod", "set the period to measure the kinetic energy increase"))
    , d_groundTruth( initData(&d_groundTruth, false, "groundTruth", "WriteObservation for GroundTruth"))
    , d_observations( initData(&d_observations, false, "observations", "write state for observationsr"))
    , d_indices ( initData ( &d_indices, "indices", "MechanicalObject points indices to monitor" ) )

    , mmodel(NULL)
    , outfile(NULL)
    , nextTime(0)
    , lastTime(0)
    , kineticEnergyThresholdReached(false)
    , timeToTestEnergyIncrease(0)
    , savedKineticEnergy(0)
{
    this->f_listening.setValue(true);
}


WriteObservation::~WriteObservation()
{
    if (outfile)
        delete outfile;
}


void WriteObservation::init()
{
    time_prec= -1.0;
    mmodel = this->getContext()->getMechanicalState();

    // test the size and range of the DOFs to write in the file output
    if (mmodel)
    {
        timeToTestEnergyIncrease = f_keperiod.getValue();
    }
    ///////////// end of the tests.

    const std::string& filename = f_filename.getFullPath();
    if (!filename.empty()) {
        outfile = new std::ofstream(filename.c_str());
        if (d_observations.getValue() && !d_groundTruth.getValue()){
            ( *outfile ) << "# Gnuplot File : positions of "
                         << d_indices.getValue().size() << " particle(s) Monitored"
                         <<  std::endl;
            ( *outfile ) << "# 1st Column : time, others : particle(s) number ";

            for (unsigned int i = 0; i < d_indices.getValue().size(); i++)
                ( *outfile ) << d_indices.getValue()[i] << " ";
            ( *outfile ) << std::endl;
        }
        if( !outfile->is_open() )
        {
            serr << "Error creating file "<<filename<<sendl;
            delete outfile;
            outfile = NULL;
        }

    }
}

void WriteObservation::reinit(){
    if (outfile)
        delete outfile;

    init();
}
void WriteObservation::reset()
{
    nextTime = 0;
    lastTime = 0;
    kineticEnergyThresholdReached = false;
    timeToTestEnergyIncrease = f_keperiod.getValue();
    savedKineticEnergy = 0;
}


void WriteObservation::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateEndEvent::checkEventType(event))
    {
        double time = getContext()->getTime();
        if (!mmodel) return;
        if (!outfile)
            if (kineticEnergyThresholdReached)
                return;

        // the time to measure the increase of energy is reached
        if (f_stopAt.getValue())
        {
            if (time > timeToTestEnergyIncrease)
            {
                simulation::Node *gnode = dynamic_cast<simulation::Node *>(this->getContext());
                if (!gnode->mass)
                {
                    // Error: the mechanical model has no mass
                    serr << "Error: Kinetic energy can not be computed. The mass for " << mmodel->getName() << " has no been defined" << sendl;
                    exit(EXIT_FAILURE);
                }
                else
                {
                    // computes the energy increase
                    if (fabs(gnode->mass->getKineticEnergy() - savedKineticEnergy) < f_stopAt.getValue())
                    {
                        sout << "WriteObservation has been stopped. Kinetic energy threshold has been reached" << sendl;
                        kineticEnergyThresholdReached = true;
                    }
                    else
                    {
                        // save the last energy measured
                        savedKineticEnergy = gnode->mass->getKineticEnergy();
                        // increase the period to measure the energy
                        timeToTestEnergyIncrease+=f_keperiod.getValue();
                    }
                }
            }
        }
        bool writeCurrent = false;
        if (nextTime<f_time.getValue().size())
        {
            // store the actual time instant
            lastTime = f_time.getValue()[nextTime];
            if (time >= lastTime) // if the time simulation is >= that the actual time instant
            {
                writeCurrent = true;
                nextTime++;
            }
        }
        else
        {
            // write the state using a period
            if (time >= (lastTime + f_period.getValue()))
            {
                writeCurrent = true;
                lastTime += f_period.getValue();
            }
        }
        if (writeCurrent)
        {

//            if (d_groundTruth.getValue())
//                time=time+this->getContext()->getDt();

            if (outfile)
            {
                if (d_observations.getValue()){
                    if (time!= time_prec){
                        (*outfile) << time << "  ";
                        if (f_writeX.getValue())
                        {
                            mmodel->writeVec(core::VecId::position(), *outfile);
                            (*outfile) << "\n";
                        }
                        time_prec =time;
                    }
                } else {
                     // write the X state
                    (*outfile) << "T= "<< time << "\n";
                    if (f_writeX.getValue())
                    {
                        (*outfile) << "  X= ";
                        mmodel->writeVec(core::VecId::position(), *outfile);
                        (*outfile) << "\n";
                    }
                }
                if (f_writeX0.getValue())
                {
                    (*outfile) << "  X0= ";
                    mmodel->writeVec(core::VecId::restPosition(), *outfile);
                    (*outfile) << "\n";
                }
                //write the V state
                if (f_writeV.getValue())
                {
                    (*outfile) << "  V= ";
                    mmodel->writeVec(core::VecId::velocity(), *outfile);
                    (*outfile) << "\n";
                }
                //write the F state
                if (f_writeF.getValue())
                {
                    (*outfile) << "  F= ";
                    mmodel->writeVec(core::VecId::force(), *outfile);
                    (*outfile) << "\n";
                }
                outfile->flush();
            }
        }
    }
}


} // namespace misc

} // namespace component

} // namespace sofa

#endif
