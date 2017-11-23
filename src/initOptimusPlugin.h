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
#ifndef INITOptimusPlugin_H
#define INITOptimusPlugin_H


#include <sofa/helper/system/config.h>
#include <pthread.h> /* pthread_t, pthread_barrier_t */
#include <sys/errno.h>

#ifdef __APPLE__
typedef int pthread_barrierattr_t;
typedef struct
{
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    int count;
    int tripCount;
} pthread_barrier_t;

#endif

#ifdef SOFA_BUILD_OPTIMUSPLUGIN
#define SOFA_OptimusPlugin_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_OptimusPlugin_API  SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#ifdef SOFA_BUILD_OPTIMUSPLUGIN
#define SOFA_STOCHASTIC_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_STOCHASTIC_API  SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#define PRNW(ARG) std::cout << "[" << this->getName() << "] WARNING: " << ARG << std::endl;
#define PRNE(ARG) std::cerr << "[" << this->getName() << "] ERROR: " << ARG << std::endl;
#define PRNS(ARG) if (this->verbose.getValue()) { std::cout << "[" << this->getName() << "]: " << ARG << std::endl; }
#define PRNSC(ARG) if (this->verbose.getValue()) { std::cout << "[" << this->getName() << "]: " << ARG; }

#define TIC this->startTime = double(this->timer->getTime());
#define TOC(arg) this->stopTime = double(this->timer->getTime()); \
                 std::cout <<  "[" << this->getName() << "] WTIME: " << arg << " " << this->stopTime - this->startTime << std::endl;

#define TOCTIC(arg) this->stopTime = double(this->timer->getTime()); \
                 std::cout << "[" << this->getName() << "] WTIME: " << arg << " " << this->stopTime - this->startTime << std::endl; \
                 this->startTime = double(this->timer->getTime());

#define SHOW_SIZE(name, A) std::cout << "Size of " << name << ": " << A.rows() << " x " << A.cols() << std::endl;

#define asumSMat(NAME, MAT) { double sm = 0.0; \
for (size_t i = 0; i < MAT.GetM(); i++) \
    for (size_t j = 0; j < MAT.GetN(); j++) \
        sm += fabs(MAT(i,j)); \
    std::cout << "@@@ ABS. SUM OF " << NAME << ": " << sm << std::endl; \
} \

#define asumSVec(NAME, VEC) { double sm = 0.0; \
    for (size_t i = 0; i < VEC.GetM(); i++) \
        sm += fabs(VEC(i)); \
    std::cout << "@@@ ABS. SUM OF " << NAME << ": " << sm << std::endl; \
} \

#define asumEMat(NAME, MAT) { double sm = 0.0; \
for (size_t i = 0; i < MAT.rows(); i++) \
    for (size_t j = 0; j < MAT.cols(); j++) \
        sm += fabs(MAT(i,j)); \
    std::cout << "@@@ ABS. SUM OF " << NAME << ": " << sm << std::endl; \
} \

#define asumEVec(NAME, VEC) { double sm = 0.0; \
    for (size_t i = 0; i < VEC.rows(); i++) \
        sm += fabs(VEC(i)); \
    std::cout << "@@@ ABS. SUM OF " << NAME << ": " << sm << std::endl; \
} \

/** \mainpage
  This plugin contains a set of Optimization Methods (initially mainly Bayesian Filtering techniques)
  */

typedef enum FilterType {
    UNDEF = -1,
    CLASSIC = 0,
    REDORD = 1,
    SIMCORR = 2
} FilterType;

#endif // INITOptimusPlugin_H
