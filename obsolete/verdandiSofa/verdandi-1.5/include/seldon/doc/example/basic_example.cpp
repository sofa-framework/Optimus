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
#define SELDON_DEBUG_LEVEL_3

#include "Seldon.hxx"
using namespace Seldon;

int main()
{

  TRY;

  Matrix<double> A(3, 3);
  Vector<double> U, V;

  A.SetIdentity();
  A(0, 1) = -1.0;

  U.Reallocate(A.GetN());
  U.Fill();

  U.Print();

  V.Reallocate(A.GetM());
  Mlt(2.0, A, U, V);

  cout << V << endl;

  END;

  return 0;

}
