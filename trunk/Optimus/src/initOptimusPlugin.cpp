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
*                               SOFA :: Plugins                               *
*                                                                             *
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
                SOFA_OptimusPlugin_API void initExternalModule();
                SOFA_OptimusPlugin_API const char* getModuleName();
                SOFA_OptimusPlugin_API const char* getModuleVersion();
                SOFA_OptimusPlugin_API const char* getModuleLicense();
                SOFA_OptimusPlugin_API const char* getModuleDescription();
                SOFA_OptimusPlugin_API const char* getModuleComponentList();
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
    return "Extended Kalman filter";
	}

	const char* getModuleVersion()
	{
        return "0.1";
	}

	const char* getModuleLicense()
	{
		return "LGPL";
	}


	const char* getModuleDescription()
	{
        return "Bayesian Filtering is a probabilistic technique for data fusion. The technique combines a concise mathematical formulation of a system with observations of that system. Probabilities are used to represent the state of a system, and likelihood functions to represent their relationships";
	}

	const char* getModuleComponentList()
	{
    return "Filter_exception; ";
	}



} 

} 

SOFA_LINK_CLASS(KalmanFilter)
SOFA_LINK_CLASS(OptimParams)
SOFA_LINK_CLASS(TestingParams)
SOFA_LINK_CLASS(BubblePackingForceField)
//SOFA_LINK_CLASS(VerdandiAnimationLoop)
//SOFA_LINK_CLASS(SofaModelWrapper)
