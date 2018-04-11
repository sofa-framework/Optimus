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
#ifndef SOFA_COMPONENT_MISC_StochasticPositionHandler_INL
#define SOFA_COMPONENT_MISC_StochasticPositionHandler_INL

#include <genericComponents/StochasticPositionHandler.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include "FilterEvents.h"

#include <fstream>
#include <sstream>

namespace sofa
{

namespace component
{

namespace misc
{


StochasticPositionHandler::StochasticPositionHandler()
    : f_filename( initData(&f_filename, "filename", "output file name"))
    , f_writeX( initData(&f_writeX, true, "writeX", "flag enabling output of X vector"))
    , d_groundTruth( initData(&d_groundTruth, false, "groundTruth", "StochasticPositionHandler for GroundTruth"))
    , d_observations( initData(&d_observations, false, "observations", "write state for observationsr"))
    , d_indices ( initData ( &d_indices, "indices", "MechanicalObject points indices to monitor" ) )
    , d_finalStochPos( initData(&d_finalStochPos, false, "finalStochPos", "StochasticPositionHandler for GroundTruth"))

    , mmodel(NULL)
    , outfile(NULL)
    , nextTime(0)
    , lastTime(0)
{
    this->f_listening.setValue(true);
}


StochasticPositionHandler::~StochasticPositionHandler()
{
    if (outfile)
        delete outfile;
}


void StochasticPositionHandler::init()
{
    time_prec= -1.0;
    mmodel = this->getContext()->getMechanicalState();


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

            ( *outfile ) << "# nParticles : "
                         << d_indices.getValue().size() << std::endl;

        }
        if( !outfile->is_open() )
        {
            serr << "Error creating file "<<filename<<sendl;
            delete outfile;
            outfile = NULL;
        }

    }
}

void StochasticPositionHandler::reinit(){
    if (outfile)
        delete outfile;

    init();
}
void StochasticPositionHandler::reset()
{
    nextTime = 0;
    lastTime = 0;
}


void StochasticPositionHandler::handleEvent(sofa::core::objectmodel::Event* event)
{
        if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateEndEvent::checkEventType(event))

//    if (sofa::component::stochastic::CorrectionEndEvent::checkEventType(event))
    {

            double time = getContext()->getTime();
            if (d_groundTruth.getValue())
                time=time+this->getContext()->getDt();
            if (!mmodel) return;
            if (outfile)
            {
                if (d_observations.getValue()){
                        (*outfile) << time << "  ";
                        if (f_writeX.getValue())
                        {
                            mmodel->writeVec(core::VecId::position(), *outfile);
                            (*outfile) << "\n";
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
                outfile->flush();
        }
    }
}

//void StochasticPositionHandler::handleEvent(sofa::core::objectmodel::Event* event)
//{
//    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateEndEvent::checkEventType(event))
//    {
//        double time = getContext()->getTime();
//        if (!mmodel) return;
//        if (d_groundTruth.getValue())
//            time=time+this->getContext()->getDt();

//        if(!d_finalStochPos.getValue()) {
//            std::cout<<"HERE FALSE"<<std::endl;

//        if (outfile)
//        {
//            if (d_observations.getValue()){
//                if (time!= time_prec){
//                    (*outfile) << time << "  ";
//                    if (f_writeX.getValue())
//                    {
//                        mmodel->writeVec(core::VecId::position(), *outfile);
//                        (*outfile) << "\n";
//                    }
//                    time_prec =time;
//                }
//            } else {
//                // write the X state
//                (*outfile) << "T= "<< time << "\n";
//                if (f_writeX.getValue())
//                {
//                    (*outfile) << "  X= ";
//                    mmodel->writeVec(core::VecId::position(), *outfile);
//                    (*outfile) << "\n";
//                }
//            }
//            outfile->flush();
//        }
//        }else{
//            return;
//        }
//    }
//}

} // namespace misc

} // namespace component

} // namespace sofa

#endif
