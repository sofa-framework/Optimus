/******************************************************************************
  *  implementation of handling and detecting events from geomagic device
 *****************************************************************************/

/* include files */
#include <fstream>
#include "GeoListener.h"




namespace sofa {

namespace component {

namespace behavior {


/**
 *        GeoListener::GeoListener
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
template <class DataTypes>
GeoListener<DataTypes>::GeoListener()
    : d_geomagicButtonPressed( initData(&d_geomagicButtonPressed, "geomagicButtonPressed", "geomagic device button pressed") )
    , d_geomagicSecondButtonPressed( initData(&d_geomagicSecondButtonPressed, "geomagicSecondButtonPressed", "geomagic device second button pressed") )
    , d_devicePosition( initData(&d_devicePosition, "geomagicPosition", "geomagic device position") )
    , d_deviceLastPosition( initData(&d_deviceLastPosition, "geomagicLastPosition", "geomagic previous device position") )
    , d_detachStiffSpring( initData(&d_detachStiffSpring, "detachSpring", "spring detached") )
    , d_savePression( initData(&d_savePression, (bool) false, "saveAttachmentData", "set to true to save attachment button pressing"))
    , d_pressFilename( initData(&d_pressFilename, "filename", "output attachment file name"))

{
    this->f_listening.setValue(true);
}



/**
 *        GeoListener::init
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
template <class DataTypes>
void GeoListener<DataTypes>::init()
{
    m_buttonWasPressed = d_geomagicButtonPressed.getValue();
    m_secondButtonWasPressed = d_geomagicSecondButtonPressed.getValue();

    sofa::core::objectmodel::BaseContext* currentContext = this->getContext();
    currentContext->get(this->m_mstate);
    currentContext->get(this->m_spring);
    currentContext->get(this->m_mapping);

    m_spring->isCompliance.setValue(true);

    // set initial device position
    //d_deviceLastPosition.setValue(d_devicePosition.getValue());
}



/**
 *        GeoListener::handleEvent
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
template <class DataTypes>
void GeoListener<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::simulation::AnimateEndEvent::checkEventType(event)) {
        if ((d_geomagicSecondButtonPressed.getValue() && !m_secondButtonWasPressed) ||
            (!d_geomagicSecondButtonPressed.getValue() && m_secondButtonWasPressed)) {

            m_spring->isCompliance.setValue(!d_geomagicSecondButtonPressed.getValue());

            m_secondButtonWasPressed = d_geomagicSecondButtonPressed.getValue();

            reinit();
        }


        //if (d_geomagicButtonPressed.getValue() && !m_buttonWasPressed) {

//            // rewrite sphere position
//            helper::ReadAccessor< Data<helper::vector<defaulttype::Vec3d>> > x_rA = m_mstate->read(core::VecCoordId::position());
//            defaulttype::Vec3d center = x_rA[0];
//            for (unsigned int i = 1; i < x_rA.size(); i++)
//            {
//                center += x_rA[i];
//            }
//            center /= x_rA.size();

//            // recalculate geomagic position
//            defaulttype::Vec3d difference = defaulttype::Vec3d(d_devicePosition.getValue().getCenter().x(),
//                 d_devicePosition.getValue().getCenter().y(), d_devicePosition.getValue().getCenter().z()) - center;

//            helper::WriteAccessor< Data<helper::vector<defaulttype::Vec3d>> > x_wA = m_mstate->write(core::VecCoordId::position());
//            for (unsigned int i = 0; i < x_wA.size(); i++)
//            {
//                x_wA[i] += difference;
//            }

        //    reinit();
        //    m_mapping->reinit();
        //}

        if (d_savePression.getValue()) {
            sofa::core::objectmodel::BaseContext* currentContext = this->getContext();
            double time = currentContext->getTime();
            std::ofstream outputFile;
            outputFile.open(d_pressFilename.getFullPath(), std::fstream::app);
            outputFile << time << "\t";
            outputFile << (d_geomagicSecondButtonPressed.getValue() ? "1" : "0");
            outputFile << std::endl;
            outputFile.close();
        }

        m_buttonWasPressed = d_geomagicButtonPressed.getValue();
    }
}



/**
 *        GeoListener::cleanup
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
template <class DataTypes>
void GeoListener<DataTypes>::cleanup()
{

}



/**
 *        GeoListener::~GeoListener
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
template <class DataTypes>
GeoListener<DataTypes>::~GeoListener()
{

}


} // namespace bahavior

} // namespace compoinent

} // namespace sofa

