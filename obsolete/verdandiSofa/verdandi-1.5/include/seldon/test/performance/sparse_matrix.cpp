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
#define SELDON_DEBUG_LEVEL_2

#include <ctime>

#include "Seldon.hxx"
#include "SeldonSolver.hxx"
using namespace Seldon;

int main(int argc, char *argv[])
{

  typedef double real;

  int i, j, l, m, n;
  real v;
  clock_t start, end;


  /////////////////
  // CONVERSIONS //
  /////////////////


  cout << "* Conversion to RowSparse" << endl;

  m = 20000;
  n = 10000;
  int Nelement = 100000;

  // First an ArrayRowSparse matrix is built so that the row and column
  // indexes include no duplicates.
  Matrix<real, General, ArrayRowSparse> A_array(m, n);
  for (l = 0; l < Nelement; l++)
    {
      i = rand() % m;
      j = rand() % n;
      v = real(rand());
      A_array.AddInteraction(i, j, v);
    }
  Vector<int> row_index_copy, col_index_copy;
  Vector<real> value_copy;
  ConvertMatrix_to_Coordinates(A_array, row_index_copy, col_index_copy,
                               value_copy);

  Vector<int> row_index, col_index;
  Vector<real> value;

  Matrix<real, General, RowSparse> A(m, n);

  start = clock();

  for (int i = 0; i < 2; i++)
    {
      row_index = row_index_copy;
      col_index = col_index_copy;
      value = value_copy;
      ConvertMatrix_from_Coordinates(row_index, col_index, value, A);
    }

  end = clock();

  cout << "CPU time: " << double(end - start) / CLOCKS_PER_SEC << endl;

  return 0;
}
