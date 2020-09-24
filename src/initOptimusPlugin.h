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


#include <sofa/helper/config.h>
#include <pthread.h> /* pthread_t, pthread_barrier_t */
#include <sys/errno.h>
#include <sofa/helper/rmath.h>

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

//#define SQR(ARG) ((ARG)*(ARG))

#ifdef SOFA_BUILD_OPTIMUSPLUGIN
#define SOFA_OPTIMUSPLUGIN_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_OPTIMUSPLUGIN_API  SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#ifdef SOFA_BUILD_OPTIMUSPLUGIN
#define SOFA_STOCHASTIC_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_STOCHASTIC_API  SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

#define PRNW(ARG) msg_warning(this) << ARG;
#define PRNE(ARG) msg_error(this) << ARG;
#define PRNS(ARG) msg_info(this) << ARG;
#define PRNSC(ARG) msg_info(this) << ARG;

//#define PRNW(ARG) std::cout << "[" << this->getName() << "] WARNING: " << ARG << std::endl;
//#define PRNE(ARG) std::cerr << "[" << this->getName() << "] ERROR: " << ARG << std::endl;
//#define PRNS(ARG) if (this->verbose.getValue()) { std::cout << "[" << this->getName() << "]: " << ARG << std::endl; }
//#define PRNSC(ARG) if (this->verbose.getValue()) { std::cout << "[" << this->getName() << "]: " << ARG; }

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

typedef enum FilterKind {
    UNDEF = -1,
    CLASSIC = 0,
    REDORD = 1,
    SIMCORR = 2
} FilterKind;
