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
#define VERDANDI_LOG_IS_ACTIVE false
#define VERDANDI_LOG_FILENAME "verdandi-%{D}.log"

#include "Verdandi.hxx"

using namespace Verdandi;

#include "Logger.cxx"


//! This class is a test class to run an example.
class ClassTest
{
public:
    //! Returns the name of the class.
    /*!
      \return The name of the class.
    */
    string GetName() const
    {
        return "ClassTest";
    }


    //! Calls the logger with an "ok" message.
    void MemberFunction()
    {
        Logger::Log(*this, "ok");
    }
};


int main(int argc, char** argv)
{
    TRY;

    Logger::Log<5>("TEST 1", "ok");

    Logger::Activate();

    Logger::Log<5>("TEST 2", "ok");

    Logger::SetOption(Logger::stdout_ | Logger::file_, true);

    Logger::Log<5>("TEST 3", "ok");

    Logger::Log<-5>("TEST 4", "ok");

    Logger::Command("hline", "-", Logger::file_);

    Logger::InitializeOptions();

    ClassTest test;
    test.MemberFunction();

    END;

    return 0;
}
