/******************************************************************************
  *  implementation of handling and detecting events from geomagic device
 *****************************************************************************/
#pragma once

/* include files */
#include <fstream>
#include "GeoEmulator.h"




namespace sofa {

namespace component {

namespace behavior {


/**
 *        GeoEmulator::GeoEmulator
 * <p>
 *   description:
 *       constructor
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
GeoEmulator::GeoEmulator()
    : d_devicePosition(initData(&d_devicePosition, "positionDevice", "position of the base of the device"))
    , d_firstButton(initData(&d_firstButton,"button1","State of the first button") )
    , d_secondButton(initData(&d_secondButton,"button2","State of the second button") )
    , d_positionSourceFilename( initData(&d_positionSourceFilename, "positionFilename", "file with data about attachments"))
    , d_buttonSourceFilename( initData(&d_buttonSourceFilename, "buttonFilename", "file with data about attachments"))
    , timeData( initData(&timeData, "timedata", "loaded time data in emulator"))
{
    this->f_listening.setValue(true);
}



/**
 *        GeoEmulator::init
 * <p>
 *   description:
 *     initialise dynamic variables
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void GeoEmulator::init()
{
    //d_devicePosition.setValue();
    d_secondButton.setValue(0);
    m_currentIndex = 0;

    parseSourcePositionsFile();
    if (d_buttonSourceFilename.getValue().size() > 0) {
        parseSourceButtonFile();
    }
}



/**
 *        GeoEmulator::bwdInit
 * <p>
 *   description:
 *     backward initialisation
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void GeoEmulator::bwdInit()
{

}



/**
 *        GeoEmulator::parseSourcePositionsFile
 * <p>
 *   description:
 *     extract positions data from source
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void GeoEmulator::parseSourcePositionsFile()
{
    helper::WriteAccessor<Data<helper::vector<double>>> time = timeData;

#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif

    time.clear();
    m_devicePositions.clear();

    std::ifstream positionsFile(d_positionSourceFilename.getFullPath());
    std::string line;
    size_t nLine = 0, dataSize, dimension;
    size_t nParticles = 0;

    if (positionsFile) {
        /// parse the header of a monitor-generated file:
        for (size_t headIndex = 0; headIndex < 2; headIndex++) {
            std::getline(positionsFile, line);
            nLine++;
            //std::cout << "Here: " << line << std::endl;
            if (line[0] != '#') {
                std::cout << " On line " << nLine << " in " << std::endl;
                return;
            }
        }

        std::stringstream ss(line);
        std::istream_iterator<std::string> it(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> tokens(it, end);

        // get amount of particles in file
        dataSize = tokens.size();
        int tki = 0;
        while (std::strcmp(tokens[tki++].c_str(),"number")!=0) ;
        nParticles = dataSize - tki;

        std::getline(positionsFile, line);
        nLine++;
        ss = std::stringstream(line);
        it = std::istream_iterator<std::string>(ss);
        tokens = std::vector<std::string>(it, end);
        dataSize = tokens.size();
        dimension = (tokens.size() - 1) / nParticles;


        while (dataSize > 1) {

            double dt;
            dt = atof(tokens[0].c_str());
            time.push_back(dt);

            helper::vector<Coord> position(nParticles);
            for (size_t i = 0; i < nParticles; i++) {
                for (size_t d = 0; d < dimension; d++) {
                    position[i][d] = atof(tokens[dimension*i+d+1].c_str());
                }
            }
            m_devicePositions.push_back(position);

            std::getline(positionsFile, line);
            nLine++;

            ss = std::stringstream(line);
            it = std::istream_iterator<std::string>(ss);
            tokens = std::vector<std::string>(it, end);
            dataSize = tokens.size();
        }
    }

    positionsFile.close();

    // set initial value
    d_devicePosition.setValue(Coord(m_devicePositions[m_currentIndex][0]));
}



/**
 *        GeoEmulator::parseSourceButtonFile
 * <p>
 *   description:
 *     extract all data from source
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void GeoEmulator::parseSourceButtonFile()
{
    helper::ReadAccessor<Data<helper::vector<double>>> time = timeData;

#ifdef __APPLE__
    setlocale(LC_ALL, "C");
#else
    std::setlocale(LC_ALL, "C");
#endif

    m_deviceSecondButton.clear();

    std::ifstream buttonFile(d_buttonSourceFilename.getFullPath());
    std::string line;
    size_t nLine = 0, dataSize;
    size_t timePosition = 0;

    if (buttonFile) {
        std::getline(buttonFile, line);
        nLine++;

        std::stringstream ss(line);
        std::istream_iterator<std::string> it(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> tokens(it, end);
        dataSize = tokens.size();

        double currentTime;
        int pressed;

        while ((dataSize > 1) || (timePosition < time.size())) {

            currentTime = atof(tokens[0].c_str());
            while (time[timePosition] < currentTime) {
                m_deviceSecondButton.push_back(0);
                timePosition++;
            }

            if (fabs(time[timePosition] - currentTime) < 1e-08) {
                pressed = atoi(tokens[1].c_str());
                m_deviceSecondButton.push_back(pressed);
                timePosition++;
            }


            std::getline(buttonFile, line);
            nLine++;

            ss = std::stringstream(line);
            it = std::istream_iterator<std::string>(ss);
            tokens = std::vector<std::string>(it, end);
            dataSize = tokens.size();
        }
    }

    buttonFile.close();

    // set initial value
    d_secondButton.setValue(0);
}



/**
 *        GeoEmulator::handleEvent
 * <p>
 *   description:
 *     a methood that does something
 * @see none
 *
 *  arguments:
 * @param  event - a pointer to some event
 *
 * @return  none
 *
 */
void GeoEmulator::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::simulation::AnimateEndEvent::checkEventType(event)) {
        helper::ReadAccessor<Data<helper::vector<double>>> time = timeData;

        sofa::core::objectmodel::BaseContext* currentContext = this->getContext();
        double currentTime = currentContext->getTime();

        if (m_currentIndex < time.size()) {
            while (currentTime > time[m_currentIndex] && m_currentIndex < time.size()) {
                m_currentIndex++;
            }

            d_devicePosition.setValue(m_devicePositions[m_currentIndex][0]);

            if (d_buttonSourceFilename.getValue().size() > 0) {
                d_secondButton.setValue(m_deviceSecondButton[m_currentIndex]);
            }
        } else {
            d_devicePosition.setValue(m_devicePositions[time.size() - 1][0]);

            if (d_buttonSourceFilename.getValue().size() > 0) {
                d_secondButton.setValue(m_deviceSecondButton[time.size() - 1]);
            }
        }
    }
}



/**
 *        GeoEmulator::cleanup
 * <p>
 *   description:
 *       executing function in exit
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
void GeoEmulator::cleanup()
{

}



/**
 *        GeoEmulator::~GeoEmulator
 * <p>
 *   description:
 *       destructor
 * @see none
 *
 *  arguments:
 * @param  none
 *
 * @return  none
 *
 */
GeoEmulator::~GeoEmulator()
{

}



} // namespace bahavior

} // namespace component

} // namespace sofa
