/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef TIMEPROFILING_H_
#define TIMEPROFILING_H_

#include <sofa/core/objectmodel/DataFileName.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>

namespace sofa
{
namespace component
{
namespace stochastic
{

class TimeProfiling
{
public:

    TimeProfiling( ) : m_isActive(false) {
        time_t_epoch = boost::posix_time::ptime(boost::gregorian::date(1970,1,1));
    }

    ~TimeProfiling() { }

    void init(core::objectmodel::DataFileName& timeDataFile) {
        m_stepNumber = 0;
        if (timeDataFile.getValue().size() > 0) {
            m_timeDataFile = timeDataFile.getFullPath();
            m_isActive = true;
            std::ofstream outputFile;
            outputFile.open(m_timeDataFile, std::fstream::app);
            outputFile << "This file contains computation time profiling data.";
            outputFile << std::endl;
            outputFile.close();
        }
    }

    void init(core::objectmodel::DataFileName& timeDataFile, bool isActive) {
        m_isActive = isActive;
        m_stepNumber = 0;
        if (m_isActive && (timeDataFile.getValue().size() > 0)) {
            m_timeDataFile = timeDataFile.getFullPath();
            std::ofstream outputFile;
            outputFile.open(m_timeDataFile, std::fstream::app);
            outputFile << "This file contains computation time profiling data.";
            outputFile << std::endl;
            outputFile.close();
        }
    }

    void SaveStartTime() {
        if (m_isActive) {
            m_stepNumber++;
            currentMomentMicroseconds = boost::posix_time::microsec_clock::local_time() - time_t_epoch;
            std::ofstream outputFile;
            outputFile.open(m_timeDataFile, std::fstream::app);
            outputFile << "Iteration step: " << m_stepNumber << std::endl;
            outputFile << "Start time: " << currentMomentMicroseconds.total_microseconds() << std::endl;
            outputFile.close();
        }
    }

    void SaveTime(std::string name) {
        if (m_isActive) {
            currentMomentMicroseconds = boost::posix_time::microsec_clock::local_time() - time_t_epoch;
            std::ofstream outputFile;
            outputFile.open(m_timeDataFile, std::fstream::app);
            outputFile << "Before" << name << "Time: " << currentMomentMicroseconds.total_microseconds() << std::endl;
            outputFile.close();
        }
    }

    void SaveEndTime() {
        if (m_isActive) {
            currentMomentMicroseconds = boost::posix_time::microsec_clock::local_time() - time_t_epoch;
            std::ofstream outputFile;
            outputFile.open(m_timeDataFile, std::fstream::app);
            outputFile << "End time: " << currentMomentMicroseconds.total_microseconds() << std::endl;
            outputFile.close();
        }
    }

protected:
    boost::posix_time::time_duration currentMomentMicroseconds;
    boost::posix_time::ptime time_t_epoch;
    bool m_isActive;
    size_t m_stepNumber;
    std::string m_timeDataFile;
};



}   /// stochastic

}   /// components

}   /// sofa


#endif /// TIMEPROFILING_H_
