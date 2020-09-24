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
#pragma once

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

