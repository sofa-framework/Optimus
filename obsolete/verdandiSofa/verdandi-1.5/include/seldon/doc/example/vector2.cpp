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
#define SELDON_DEBUG_LEVEL_4
#include "Seldon.hxx"
using namespace Seldon;

#include "vector/Vector2.cxx"

int main()
{

  TRY;

  Vector<int> length(3);
  length(0) = 2;
  length(1) = 3;
  length(2) = 7;
  Vector2<double> V(length);

  // Fills all inner vectors with 2.
  V.Fill(2.);
  // Access to the second inner vector, to fill it with 5.
  V(1).Fill(5.);

  V.Print();
  cout << "First element of the second inner vector: " << V(1, 0) << endl;
  // Note that V(1)(0) would have returned the same element.

  Vector<double> inner_vector(4);
  inner_vector.Fill();
  // Appends a new inner vector.
  V.PushBack(inner_vector);
  V.Print();

  cout << "After setting to -10 the second element of the last inner vector:"
       << endl;
  V(3, 1) = -10.;
  V.Print();

  END;

  return 0;

}
