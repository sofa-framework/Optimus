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
#include "initOptimusPlugin.h"



namespace sofa
{

namespace component
{

    //Here are just several convenient functions to help user to know what contains the plugin
	extern "C" {
                SOFA_OPTIMUSPLUGIN_API void initExternalModule();
                SOFA_OPTIMUSPLUGIN_API const char* getModuleName();
                SOFA_OPTIMUSPLUGIN_API const char* getModuleVersion();
                SOFA_OPTIMUSPLUGIN_API const char* getModuleLicense();
                SOFA_OPTIMUSPLUGIN_API const char* getModuleDescription();
                SOFA_OPTIMUSPLUGIN_API const char* getModuleComponentList();
	}


	void initExternalModule()
	{
		static bool first = true;
		if (first)
		{
			first = false;
		}
	}


	const char* getModuleName()
	{
        return "Optimus";
	}


	const char* getModuleVersion()
	{
        return "1.0";
	}


	const char* getModuleLicense()
	{
		return "GPL";
	}


	const char* getModuleDescription()
	{
        return "Bayesian Filtering is a probabilistic technique for data fusion. The technique combines a concise mathematical formulation of a system with observations of that system. Probabilities are used to represent the state of a system, and likelihood functions to represent their relationships";
	}


	const char* getModuleComponentList()
	{
        return "Filter_exception";
	}


} // namespace component

} // na,espace sofa



//#ifdef __APPLE__
//int pthread_barrier_init(pthread_barrier_t *barrier, const pthread_barrierattr_t *attr, unsigned int count)
//{
//    if(count == 0)
//    {
//        errno = EINVAL;
//        return -1;
//    }
//    if(pthread_mutex_init(&barrier->mutex, 0) < 0)
//    {
//        return -1;
//    }
//    if(pthread_cond_init(&barrier->cond, 0) < 0)
//    {
//        pthread_mutex_destroy(&barrier->mutex);
//        return -1;
//    }
//    barrier->tripCount = count;
//    barrier->count = 0;
//
//    return 0;
//}

//int pthread_barrier_destroy(pthread_barrier_t *barrier)
//{
//    pthread_cond_destroy(&barrier->cond);
//    pthread_mutex_destroy(&barrier->mutex);
//    return 0;
//}

//int pthread_barrier_wait(pthread_barrier_t *barrier)
//{
//    pthread_mutex_lock(&barrier->mutex);
//    ++(barrier->count);
//    if(barrier->count >= barrier->tripCount)
//    {
//        barrier->count = 0;
//        pthread_cond_broadcast(&barrier->cond);
//        pthread_mutex_unlock(&barrier->mutex);
//        return 1;
//    }
//    else
//    {
//        pthread_cond_wait(&barrier->cond, &(barrier->mutex));
//        pthread_mutex_unlock(&barrier->mutex);
//        return 0;
//    }
//}
//#endif



SOFA_LINK_CLASS(OptimParams)

//SOFA_LINK_CLASS(KalmanFilter)
//SOFA_LINK_CLASS(TestingParams)
//SOFA_LINK_CLASS(BubblePackingForceField)

//#ifdef SOFA_HAVE_VERDANDI
//SOFA_LINK_CLASS(VerdandiAnimationLoop)
//SOFA_LINK_CLASS(SofaModelWrapper)

//SOFA_LINK_CLASS(SofaLinearObservationManager)
//SOFA_LINK_CLASS(MappedPointsObservationManager)
//SOFA_LINK_CLASS(ARObservationManager)

//SOFA_LINK_CLASS(SimulatedStateObservationSource)

//SOFA_LINK_CLASS(SofaReducedOrderUKF)
//SOFA_LINK_CLASS(SofaUnscentedKalmanFilter)
//SOFA_LINK_CLASS(SofaForwardDriver)
//#endif

