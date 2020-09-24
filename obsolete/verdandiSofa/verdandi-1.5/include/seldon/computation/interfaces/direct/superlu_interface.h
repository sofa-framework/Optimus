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
#ifndef __SUPERLU_INTERFACE /* allow multiple inclusions */
#define __SUPERLU_INTERFACE

#include "slu_zdefs.h"

/*! \brief Supernodal LU factor related */
extern void
dCreate_CompCol_Matrix(SuperMatrix *, int, int, int, double *,
		       int *, int *, Stype_t, Dtype_t, Mtype_t);
extern void
dCreate_Dense_Matrix(SuperMatrix *, int, int, double *, int,
		     Stype_t, Dtype_t, Mtype_t);

extern void    dgstrf (superlu_options_t*, SuperMatrix*,
                       int, int, int*, void *, int, int *, int *,
                       SuperMatrix *, SuperMatrix *, SuperLUStat_t*, int *);

extern void    dgstrs (trans_t, SuperMatrix *, SuperMatrix *, int *, int *,
                        SuperMatrix *, SuperLUStat_t*, int *);

extern int     dQuerySpace (SuperMatrix *, SuperMatrix *, mem_usage_t *);

#endif /* __SUPERLU_INTERFACE */
