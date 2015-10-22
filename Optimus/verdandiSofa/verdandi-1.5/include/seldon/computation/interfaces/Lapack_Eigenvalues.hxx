// Copyright (C) 2012 Vivien Mallet
//
// This file is part of the linear-algebra library Seldon,
// http://seldon.sourceforge.net/.
//
// Seldon is free software; you can redistribute it and/or modify it under the
// terms of the GNU Lesser General Public License as published by the Free
// Software Foundation; either version 2.1 of the License, or (at your option)
// any later version.
//
// Seldon is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
// more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Seldon. If not, see http://www.gnu.org/licenses/.


#ifndef SELDON_FILE_LAPACK_EIGENVALUES_HXX

/*
  Functions included in this file:

  xGEEV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xSYEV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xHEEV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xSPEV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xHPEV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xSYGV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xGGEV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xHEGV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xSPGV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xHPGV   (GetEigenvalues, GetEigenvaluesEigenvectors)
  xGESVD  (GetSVD)
  xGEQRF  (GetHessenberg)
  ZGEQRF + ZUNGQR + ZUNMQR + ZGGHRD   (GetHessenberg)
  ZGEQRF + ZUNGQR + ZUNMQR + ZGGHRD + ZHGEQZ   (GetQZ)
  (SolveSylvester)
*/

namespace Seldon
{


  /////////////////////////////////
  // STANDARD EIGENVALUE PROBLEM //


  /* RowMajor */


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<float, Prop, RowMajor, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& wr,
		      Vector<float, VectFull, Allocator3>& wi,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop, RowMajor, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& wr,
				  Vector<float, VectFull, Allocator3>& wi,
				  Matrix<float, General, RowMajor, Allocator4>& zr,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, RowMajor, Allocator1>& A,
		      Vector<complex<float>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, RowMajor, Allocator1>& A,
				  Vector<complex<float>, VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<double, Prop, RowMajor, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& wr,
		      Vector<double, VectFull, Allocator3>& wi,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop, RowMajor, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& wr,
				  Vector<double, VectFull, Allocator3>& wi,
				  Matrix<double, General, RowMajor, Allocator4>& zr,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, RowMajor, Allocator1>& A,
		      Vector<complex<double>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, RowMajor, Allocator1>& A,
				  Vector<complex<double>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* ColMajor */


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<float, Prop, ColMajor, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& wr,
		      Vector<float, VectFull, Allocator3>& wi,
		      LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop, ColMajor, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& wr,
				  Vector<float, VectFull, Allocator3>& wi,
				  Matrix<float, General, ColMajor, Allocator4>&zr,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, ColMajor, Allocator1>& A,
		      Vector<complex<float>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, ColMajor, Allocator1>& A,
				  Vector<complex<float>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<double, Prop, ColMajor, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& wr,
		      Vector<double, VectFull, Allocator3>& wi,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop, ColMajor, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& wr,
				  Vector<double, VectFull, Allocator3>& wi,
				  Matrix<double, General, ColMajor, Allocator4>&zr,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, ColMajor, Allocator1>& A,
		      Vector<complex<double>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, ColMajor, Allocator1>& A,
				  Vector<complex<double>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* RowSym */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<float, Prop, RowSym, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop, RowSym, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<float, General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, RowSym, Allocator1>& A,
		      Vector<complex<float>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, RowSym, Allocator1>& A,
				  Vector<complex<float>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<double, Prop, RowSym, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop, RowSym, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<double, General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, RowSym, Allocator1>& A,
		      Vector<complex<double>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, RowSym, Allocator1>& A,
				  Vector<complex<double>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* ColSym */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<float, Prop, ColSym, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop, ColSym, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<float, General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, ColSym, Allocator1>& A,
		      Vector<complex<float>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, ColSym, Allocator1>& A,
				  Vector<complex<float>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<double, Prop, ColSym, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop, ColSym, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<double, General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, ColSym, Allocator1>& A,
		      Vector<complex<double>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, ColSym, Allocator1>& A,
				  Vector<complex<double>,
				  VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* RowHerm */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, RowHerm, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, RowHerm, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, RowHerm, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, RowHerm, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* ColHerm */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, ColHerm, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, ColHerm, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, ColHerm, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, ColHerm, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* RowSymPacked */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<float, Prop, RowSymPacked, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop,RowSymPacked, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<float, General, RowMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>, Prop, RowSymPacked, Allocator1>& A,
		      Vector<complex<float>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);

  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>, Prop,
				  RowSymPacked, Allocator1>& A,
				  Vector<complex<float>,VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<double, Prop, RowSymPacked, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<double,Prop,RowSymPacked, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<double, General, RowMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>, Prop, RowSymPacked, Allocator1>& A,
		      Vector<complex<double>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>, Prop,
				  RowSymPacked, Allocator1>& A,
				  Vector<complex<double>,VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* ColSymPacked */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<float, Prop, ColSymPacked, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop,ColSymPacked, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<float, General, ColMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop, ColSymPacked, Allocator1>& A,
		      Vector<complex<float>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, ColSymPacked, Allocator1>& A,
				  Vector<complex<float>, VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<double, Prop, ColSymPacked, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<double,Prop,ColSymPacked, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<double, General, ColMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop, ColSymPacked, Allocator1>& A,
		      Vector<complex<double>, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, ColSymPacked, Allocator1>& A,
				  Vector<complex<double>, VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator3>& z,
				  LapackInfo& info = lapack_info);


  /* RowHermPacked */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop, RowHermPacked, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, RowHermPacked, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop, RowHermPacked, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, RowHermPacked, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  /* ColHermPacked */


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop, ColHermPacked, Allocator1>& A,
		      Vector<float, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop, ColHermPacked, Allocator1>& A,
				  Vector<float, VectFull, Allocator2>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop, ColHermPacked, Allocator1>& A,
		      Vector<double, VectFull, Allocator2>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop, class Allocator1, class Allocator2, class Allocator3>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop, ColHermPacked, Allocator1>& A,
				  Vector<double, VectFull, Allocator2>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator3>&z,
				  LapackInfo& info = lapack_info);


  // STANDARD EIGENVALUE PROBLEM //
  /////////////////////////////////


  ////////////////////////////////////
  // GENERALIZED EIGENVALUE PROBLEM //


  /* RowSym */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<float, Prop1, RowSym, Allocator1>& A,
		      Matrix<float, Prop2, RowSym, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop1, RowSym, Allocator1>& A,
				  Matrix<float, Prop2, RowSym, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<float, General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<float>, Prop1, RowSym, Allocator1>& A,
		      Matrix<complex<float>, Prop2, RowSym, Allocator2>& B,
		      Vector<complex<float>, VectFull, Allocator4>& alpha,
		      Vector<complex<float>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, RowSym, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, RowSym, Allocator2>& B,
				  Vector<complex<float>,
				  VectFull, Allocator4>& alpha,
				  Vector<complex<float>,
				  VectFull, Allocator5>& beta,
				  Matrix<complex<float>,
				  Prop3, RowMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<double, Prop1, RowSym, Allocator1>& A,
		      Matrix<double, Prop2, RowSym, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop1, RowSym, Allocator1>& A,
				  Matrix<double, Prop2, RowSym, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<double, General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<double>, Prop1, RowSym, Allocator1>& A,
		      Matrix<complex<double>, Prop2, RowSym, Allocator2>& B,
		      Vector<complex<double>, VectFull, Allocator4>& alpha,
		      Vector<complex<double>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, RowSym, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, RowSym, Allocator2>& B,
				  Vector<complex<double>,
				  VectFull, Allocator4>& alpha,
				  Vector<complex<double>,
				  VectFull, Allocator5>& beta,
				  Matrix<complex<double>,
				  Prop3, RowMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  /* ColSym */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<float, Prop1, ColSym, Allocator1>& A,
		      Matrix<float, Prop2, ColSym, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop1, ColSym, Allocator1>& A,
				  Matrix<float, Prop2, ColSym, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<float, General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<float>, Prop1, ColSym, Allocator1>& A,
		      Matrix<complex<float>, Prop2, ColSym, Allocator2>& B,
		      Vector<complex<float>, VectFull, Allocator4>& alpha,
		      Vector<complex<float>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Alloc1,
	   class Alloc2, class Alloc4, class Alloc5, class Alloc6>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, ColSym, Alloc1>& A,
				  Matrix<complex<float>,
				  Prop2, ColSym, Alloc2>& B,
				  Vector<complex<float>, VectFull, Alloc4>& alpha,
				  Vector<complex<float>, VectFull, Alloc5>& beta,
				  Matrix<complex<float>,
				  Prop3, ColMajor, Alloc6>& V,
				  LapackInfo& info = lapack_info);

  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<double, Prop1, ColSym, Allocator1>& A,
		      Matrix<double, Prop2, ColSym, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop1, ColSym, Allocator1>& A,
				  Matrix<double, Prop2, ColSym, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<double, General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<double>, Prop1, ColSym, Allocator1>& A,
		      Matrix<complex<double>, Prop2, ColSym, Allocator2>& B,
		      Vector<complex<double>, VectFull, Allocator4>& alpha,
		      Vector<complex<double>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Alloc1,
	   class Alloc2, class Alloc4, class Alloc5, class Alloc6>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, ColSym, Alloc1>& A,
				  Matrix<complex<double>,
				  Prop2, ColSym, Alloc2>& B,
				  Vector<complex<double>, VectFull, Alloc4>& alpha,
				  Vector<complex<double>, VectFull, Alloc5>& beta,
				  Matrix<complex<double>,
				  Prop3, ColMajor, Alloc6>& V,
				  LapackInfo& info = lapack_info);


  /* RowHerm */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<float>, Prop1, RowHerm, Allocator1>& A,
		      Matrix<complex<float>, Prop2, RowHerm, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, RowHerm, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, RowHerm, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<double>, Prop1, RowHerm, Allocator1>& A,
		      Matrix<complex<double>, Prop2, RowHerm, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, RowHerm, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, RowHerm, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  /* ColHerm */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<float>, Prop1, ColHerm, Allocator1>& A,
		      Matrix<complex<float>, Prop2, ColHerm, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, ColHerm, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, ColHerm, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<double>, Prop1, ColHerm, Allocator1>& A,
		      Matrix<complex<double>, Prop2, ColHerm, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, ColHerm, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, ColHerm, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  /* RowSymPacked */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<float, Prop1, RowSymPacked, Allocator1>& A,
		      Matrix<float, Prop2, RowSymPacked, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<float,
				  Prop1, RowSymPacked, Allocator1>& A,
				  Matrix<float,
				  Prop2, RowSymPacked, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<float,
				  General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop1, RowSymPacked, Allocator1>& A,
		      Matrix<complex<float>,
		      Prop2, RowSymPacked, Allocator2>& B,
		      Vector<complex<float>, VectFull, Allocator3>& alpha,
		      Vector<complex<float>, VectFull, Allocator4>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4, class Allocator5>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, RowSymPacked, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, RowSymPacked, Allocator2>& B,
				  Vector<complex<float>,
				  VectFull, Allocator3>& alpha,
				  Vector<complex<float>,
				  VectFull, Allocator4>& beta,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator5>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<double, Prop1, RowSymPacked, Allocator1>& A,
		      Matrix<double, Prop2, RowSymPacked, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<double,
				  Prop1, RowSymPacked, Allocator1>& A,
				  Matrix<double,
				  Prop2, RowSymPacked, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<double,
				  General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop1, RowSymPacked, Allocator1>& A,
		      Matrix<complex<double>,
		      Prop2, RowSymPacked, Allocator2>& B,
		      Vector<complex<double>, VectFull, Allocator3>& alpha,
		      Vector<complex<double>, VectFull, Allocator4>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4, class Allocator5>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, RowSymPacked, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, RowSymPacked, Allocator2>& B,
				  Vector<complex<double>,
				  VectFull, Allocator3>& alpha,
				  Vector<complex<double>,
				  VectFull, Allocator4>& beta,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator5>& z,
				  LapackInfo& info = lapack_info);


  /* ColSymPacked */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<float, Prop1, ColSymPacked, Allocator1>& A,
		      Matrix<float, Prop2, ColSymPacked, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<float,
				  Prop1, ColSymPacked, Allocator1>& A,
				  Matrix<float,
				  Prop2, ColSymPacked, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<float,
				  General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop1, ColSymPacked, Allocator1>& A,
		      Matrix<complex<float>,
		      Prop2, ColSymPacked, Allocator2>& B,
		      Vector<complex<float>, VectFull, Allocator3>& alpha,
		      Vector<complex<float>, VectFull, Allocator4>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4, class Allocator5>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, ColSymPacked, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, ColSymPacked, Allocator2>& B,
				  Vector<complex<float>,
				  VectFull, Allocator3>& alpha,
				  Vector<complex<float>,
				  VectFull, Allocator4>& beta,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator5>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<double, Prop1, ColSymPacked, Allocator1>& A,
		      Matrix<double, Prop2, ColSymPacked, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<double,
				  Prop1, ColSymPacked, Allocator1>& A,
				  Matrix<double,
				  Prop2, ColSymPacked, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<double,
				  General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop1, ColSymPacked, Allocator1>& A,
		      Matrix<complex<double>,
		      Prop2, ColSymPacked, Allocator2>& B,
		      Vector<complex<double>, VectFull, Allocator3>& alpha,
		      Vector<complex<double>, VectFull, Allocator4>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4, class Allocator5>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, ColSymPacked, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, ColSymPacked, Allocator2>& B,
				  Vector<complex<double>,
				  VectFull, Allocator3>& alpha,
				  Vector<complex<double>,
				  VectFull, Allocator4>& beta,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator5>& z,
				  LapackInfo& info = lapack_info);


  /* RowHermPacked */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop1, RowHermPacked, Allocator1>& A,
		      Matrix<complex<float>,
		      Prop2, RowHermPacked, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, RowHermPacked, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, RowHermPacked, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<complex<float>,
				  General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop1, RowHermPacked, Allocator1>& A,
		      Matrix<complex<double>,
		      Prop2, RowHermPacked, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, RowHermPacked, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, RowHermPacked, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<complex<double>,
				  General, RowMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  /* ColHermPacked */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<float>,
		      Prop1, ColHermPacked, Allocator1>& A,
		      Matrix<complex<float>,
		      Prop2, ColHermPacked, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, ColHermPacked, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, ColHermPacked, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& w,
				  Matrix<complex<float>,
				  General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3>
  void GetEigenvalues(Matrix<complex<double>,
		      Prop1, ColHermPacked, Allocator1>& A,
		      Matrix<complex<double>,
		      Prop2, ColHermPacked, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& w,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, ColHermPacked, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, ColHermPacked, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& w,
				  Matrix<complex<double>,
				  General, ColMajor, Allocator4>& z,
				  LapackInfo& info = lapack_info);


  /* RowMajor */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3,
	   class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<float, Prop1, RowMajor, Allocator1>& A,
		      Matrix<float, Prop2, RowMajor, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& alpha_real,
		      Vector<float, VectFull, Allocator4>& alpha_imag,
		      Vector<float, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator3, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop1, RowMajor, Allocator1>& A,
				  Matrix<float, Prop2, RowMajor, Allocator2>& B,
				  Vector<float, VectFull, Allocator3>& alpha_real,
				  Vector<float, VectFull, Allocator4>& alpha_imag,
				  Vector<float, VectFull, Allocator5>& beta,
				  Matrix<float, Prop3, RowMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<float>, Prop1, RowMajor, Allocator1>& A,
		      Matrix<complex<float>, Prop2, RowMajor, Allocator2>& B,
		      Vector<complex<float>, VectFull, Allocator4>& alpha,
		      Vector<complex<float>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, RowMajor, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, RowMajor, Allocator2>& B,
				  Vector<complex<float>,
				  VectFull, Allocator4>& alpha,
				  Vector<complex<float>,
				  VectFull, Allocator5>& beta,
				  Matrix<complex<float>,
				  Prop3, RowMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3,
	   class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<double, Prop1, RowMajor, Allocator1>& A,
		      Matrix<double, Prop2, RowMajor, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& alpha_real,
		      Vector<double, VectFull, Allocator4>& alpha_imag,
		      Vector<double, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator3, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop1, RowMajor, Allocator1>& A,
				  Matrix<double, Prop2, RowMajor, Allocator2>& B,
				  Vector<double, VectFull, Allocator3>& alpha_real,
				  Vector<double, VectFull, Allocator4>& alpha_imag,
				  Vector<double, VectFull, Allocator5>& beta,
				  Matrix<double, Prop3, RowMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<double>, Prop1, RowMajor, Allocator1>& A,
		      Matrix<complex<double>, Prop2, RowMajor, Allocator2>& B,
		      Vector<complex<double>, VectFull, Allocator4>& alpha,
		      Vector<complex<double>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, RowMajor, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, RowMajor, Allocator2>& B,
				  Vector<complex<double>,
				  VectFull, Allocator4>& alpha,
				  Vector<complex<double>,
				  VectFull, Allocator5>& beta,
				  Matrix<complex<double>,
				  Prop3, RowMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  /* ColMajor */


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3,
	   class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<float, Prop1, ColMajor, Allocator1>& A,
		      Matrix<float, Prop2, ColMajor, Allocator2>& B,
		      Vector<float, VectFull, Allocator3>& alpha_real,
		      Vector<float, VectFull, Allocator4>& alpha_imag,
		      Vector<float, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator3, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<float, Prop1, ColMajor, Allocator1>& A,
				  Matrix<float, Prop2, ColMajor, Allocator2>& B,
				  Vector<float,
				  VectFull, Allocator3>& alpha_real,
				  Vector<float,
				  VectFull, Allocator4>& alpha_imag,
				  Vector<float, VectFull, Allocator5>& beta,
				  Matrix<float, Prop3, ColMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<float>, Prop1, ColMajor, Allocator1>& A,
		      Matrix<complex<float>, Prop2, ColMajor, Allocator2>& B,
		      Vector<complex<float>, VectFull, Allocator4>& alpha,
		      Vector<complex<float>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<complex<float>,
				  Prop1, ColMajor, Allocator1>& A,
				  Matrix<complex<float>,
				  Prop2, ColMajor, Allocator2>& B,
				  Vector<complex<float>,
				  VectFull, Allocator4>& alpha,
				  Vector<complex<float>,
				  VectFull, Allocator5>& beta,
				  Matrix<complex<float>,
				  Prop3, ColMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator3,
	   class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<double, Prop1, ColMajor, Allocator1>& A,
		      Matrix<double, Prop2, ColMajor, Allocator2>& B,
		      Vector<double, VectFull, Allocator3>& alpha_real,
		      Vector<double, VectFull, Allocator4>& alpha_imag,
		      Vector<double, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator3, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<double, Prop1, ColMajor, Allocator1>& A,
				  Matrix<double, Prop2, ColMajor, Allocator2>& B,
				  Vector<double,
				  VectFull, Allocator3>& alpha_real,
				  Vector<double,
				  VectFull, Allocator4>& alpha_imag,
				  Vector<double, VectFull, Allocator5>& beta,
				  Matrix<double, Prop3, ColMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Allocator1,
	   class Allocator2, class Allocator4, class Allocator5>
  void GetEigenvalues(Matrix<complex<double>, Prop1, ColMajor, Allocator1>& A,
		      Matrix<complex<double>, Prop2, ColMajor, Allocator2>& B,
		      Vector<complex<double>, VectFull, Allocator4>& alpha,
		      Vector<complex<double>, VectFull, Allocator5>& beta,
		      LapackInfo& info = lapack_info);


  template<class Prop1, class Prop2, class Prop3, class Allocator1,
	   class Allocator2, class Allocator4,
	   class Allocator5, class Allocator6>
  void GetEigenvaluesEigenvectors(Matrix<complex<double>,
				  Prop1, ColMajor, Allocator1>& A,
				  Matrix<complex<double>,
				  Prop2, ColMajor, Allocator2>& B,
				  Vector<complex<double>,
				  VectFull, Allocator4>& alpha,
				  Vector<complex<double>,
				  VectFull, Allocator5>& beta,
				  Matrix<complex<double>,
				  Prop3, ColMajor, Allocator6>& V,
				  LapackInfo& info = lapack_info);


  // GENERALIZED EIGENVALUE PROBLEM //
  ////////////////////////////////////


  //////////////////////////////////
  // SINGULAR VALUE DECOMPOSITION //


  /* RowMajor */


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<float, Prop1, RowMajor, Allocator1>& A,
	      Vector<float, VectFull, Allocator4>& lambda,
	      Matrix<float, General, RowMajor, Allocator2>& u,
	      Matrix<float, General, RowMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<complex<float>, Prop1, RowMajor, Allocator1>& A,
	      Vector<float, VectFull, Allocator4>& lambda,
	      Matrix<complex<float>, General, RowMajor, Allocator2>& u,
	      Matrix<complex<float>, General, RowMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<double, Prop1, RowMajor, Allocator1>& A,
	      Vector<double, VectFull, Allocator4>& lambda,
	      Matrix<double, General, RowMajor, Allocator2>& u,
	      Matrix<double, General, RowMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<complex<double>, Prop1, RowMajor, Allocator1>& A,
	      Vector<double, VectFull, Allocator4>& lambda,
	      Matrix<complex<double>, General, RowMajor, Allocator2>& u,
	      Matrix<complex<double>, General, RowMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  /* ColMajor */


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<float, Prop1, ColMajor, Allocator1>& A,
	      Vector<float, VectFull, Allocator4>& lambda,
	      Matrix<float, General, ColMajor, Allocator2>& u,
	      Matrix<float, General, ColMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<complex<float>, Prop1, ColMajor, Allocator1>& A,
	      Vector<float, VectFull, Allocator4>& lambda,
	      Matrix<complex<float>, General, ColMajor, Allocator2>& u,
	      Matrix<complex<float>, General, ColMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<double, Prop1, ColMajor, Allocator1>& A,
	      Vector<double, VectFull, Allocator4>& lambda,
	      Matrix<double, General, ColMajor, Allocator2>& u,
	      Matrix<double, General, ColMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  template<class Prop1, class Allocator1, class Allocator2,
	   class Allocator3, class Allocator4>
  void GetSVD(Matrix<complex<double>, Prop1, ColMajor, Allocator1>& A,
	      Vector<double, VectFull, Allocator4>& sigma,
	      Matrix<complex<double>, General, ColMajor, Allocator2>& u,
	      Matrix<complex<double>, General, ColMajor, Allocator3>& v,
	      LapackInfo& info = lapack_info);


  // pseudo inverse
  template<class Prop1, class Allocator1>
  void GetPseudoInverse(Matrix<double, Prop1, ColMajor, Allocator1>& A,
			double epsilon, LapackInfo& info = lapack_info);


  // SINGULAR VALUE DECOMPOSITION //
  //////////////////////////////////


  ///////////////////////////////////
  // RESOLUTION SYLVESTER EQUATION //


  void GetHessenberg(Matrix<complex<double>, General, ColMajor>& A,
                     Matrix<complex<double>, General, ColMajor>& B,
                     Matrix<complex<double>, General, ColMajor>& Q,
                     Matrix<complex<double>, General, ColMajor>& Z);


  void GetQZ(Matrix<complex<double>, General, ColMajor>& A,
	     Matrix<complex<double>, General, ColMajor>& B,
	     Matrix<complex<double>, General, ColMajor>& Q,
	     Matrix<complex<double>, General, ColMajor>& Z);


  void GetHessenberg(Matrix<complex<double>, General, RowMajor>& A,
                     Matrix<complex<double>, General, RowMajor>& B,
                     Matrix<complex<double>, General, RowMajor>& Q,
                     Matrix<complex<double>, General, RowMajor>& Z);


  void GetQZ(Matrix<complex<double>, General, RowMajor>& A,
	     Matrix<complex<double>, General, RowMajor>& B,
	     Matrix<complex<double>, General, RowMajor>& Q,
	     Matrix<complex<double>, General, RowMajor>& Z);


  //! Gaussian elimination to solve A X = B with A an Hessenberg matrix
  template<class T, class Prop, class Storage, class Allocator, class Vector1>
  void SolveHessenberg(Matrix<T, Prop, Storage, Allocator>& A, Vector1& B);


  /*! \brief Gaussian elimination to solve A X = B with A matrix so that a_ij
    = 0 for i > j+2 */
  template<class T, class Prop, class Storage, class Allocator, class Vector1>
  void SolveHessenbergTwo(Matrix<T, Prop, Storage, Allocator>& A, Vector1& B);


  /*! \brief Solves A X B^H + C X D^H = E, E is overwritten with result X. A,
    B, C and D are modified */
  template<class Prop, class Storage, class Allocator>
  void SolveSylvester(Matrix<complex<double>, Prop, Storage, Allocator>& A,
		      Matrix<complex<double>, Prop, Storage, Allocator>& B,
		      Matrix<complex<double>, Prop, Storage, Allocator>& C,
		      Matrix<complex<double>, Prop, Storage, Allocator>& D,
		      Matrix<complex<double>, Prop, Storage, Allocator>& E);


  void GetHessenberg(Matrix<double, General, ColMajor>& A,
                     Matrix<double, General, ColMajor>& B,
                     Matrix<double, General, ColMajor>& Q,
                     Matrix<double, General, ColMajor>& Z);


  void GetQZ(Matrix<double, General, ColMajor>& A,
	     Matrix<double, General, ColMajor>& B,
	     Matrix<double, General, ColMajor>& Q,
	     Matrix<double, General, ColMajor>& Z);


  void GetHessenberg(Matrix<double, General, RowMajor>& A,
                     Matrix<double, General, RowMajor>& B,
                     Matrix<double, General, RowMajor>& Q,
                     Matrix<double, General, RowMajor>& Z);


  void GetQZ(Matrix<double, General, RowMajor>& A,
	     Matrix<double, General, RowMajor>& B,
	     Matrix<double, General, RowMajor>& Q,
	     Matrix<double, General, RowMajor>& Z);


  /*! Solves A X B^T + C X D^T = E, E is overwritten with result X.  A, B, C
    and D are modified */
  template<class Prop, class Storage, class Allocator>
  void SolveSylvester(Matrix<double, Prop, Storage, Allocator>& A,
		      Matrix<double, Prop, Storage, Allocator>& B,
		      Matrix<double, Prop, Storage, Allocator>& C,
		      Matrix<double, Prop, Storage, Allocator>& D,
		      Matrix<double, Prop, Storage, Allocator>& E);


  // RESOLUTION SYLVESTER EQUATION //
  ///////////////////////////////////


} // end namespace

#define SELDON_FILE_LAPACK_EIGENVALUES_HXX
#endif
