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


#ifndef SELDON_FILE_LAPACK_LEAST_SQUARES_HXX

/*
  Functions included in this file:

  xGEQRF   (GetQR, GetLQ)
  xGELQF   (GetQR, GetLQ)
  xGEQP3   (GetQR_Pivot)
  xORGQR   (GetQ_FromQR)
  xUNGQR   (GetQ_FromQR)
  xUNMQR   (MltQ_FromQR)
  xORMQR   (MltQ_FromQR)
  xORMQR + xTRSM   (SolveQR)
  ZUNMQR + ZTRSM   (SolveQR)
  xORMLQ + xTRSM   (SolveQR)
  ZUNMLQ + ZTRSM   (SolveQR)
  xTRSM + xORMLQ   (SolveLQ)
  ZTRSM + ZUNMLQ   (SolveLQ)
  xTRSM + xORMQR   (SolveLQ)
  ZTRSM + ZUNMQR   (SolveLQ)
*/

namespace Seldon
{


  ///////////
  // GETQR //


  /*** ColMajor ***/


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR(Matrix<float, Prop0, ColMajor, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR(Matrix<double, Prop0, ColMajor, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR(Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
	     Vector<complex<double>, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  /*** RowMajor ***/


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR(Matrix<float, Prop0, RowMajor, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR(Matrix<double, Prop0, RowMajor, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR(Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A,
	     Vector<complex<double>, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  // GETQR //
  ///////////


  /////////////////
  // GETQR_PIVOT //


  /*** ColMajor ***/


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQR_Pivot(Matrix<double, Prop0, ColMajor, Allocator0>& A,
		   Vector<double, VectFull, Allocator1>& tau,
		   Vector<int>& ipivot, LapackInfo& info = lapack_info);


  // GETQR_PIVOT //
  /////////////////


  /////////////////
  // GETQ_FROMQR //


  /*** ColMajor ***/

  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQ_FromQR(Matrix<double, Prop0, ColMajor, Allocator0>& A,
		   Vector<double, VectFull, Allocator1>& tau,
		   LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetQ_FromQR(Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
		   Vector<complex<double>, VectFull, Allocator1>& tau,
		   LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1, class Allocator2, class Side, class Trans>
  void MltQ_FromQR(const Side& side, const Trans& trans,
		   Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
		   Vector<complex<double>, VectFull, Allocator1>& tau,
		   Matrix<complex<double>, Prop0, ColMajor, Allocator2>& C,
		   LapackInfo& info = lapack_info);


  // GETQ_FROMQR //
  /////////////////


  ///////////
  // GETLQ //


  /*** ColMajor ***/


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetLQ(Matrix<float, Prop0, ColMajor, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetLQ(Matrix<double, Prop0, ColMajor, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetLQ(Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
	     Vector<complex<double>, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  /*** RowMajor ***/


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetLQ(Matrix<float, Prop0, RowMajor, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetLQ(Matrix<double, Prop0, RowMajor, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  template<class Prop0, class Allocator0,
	   class Allocator1>
  void GetLQ(Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A,
	     Vector<complex<double>, VectFull, Allocator1>& tau,
	     LapackInfo& info = lapack_info);


  // GETLQ //
  ///////////


  /////////////////
  // MLTQ_FROMQR //


  /*** ColMajor ***/


  template<class Prop0, class Allocator0,
	   class Allocator1, class Allocator2, class IsTranspose>
  void MltQ_FromQR(Matrix<double, Prop0, ColMajor, Allocator0>& A,
		   const IsTranspose& trans,
		   Vector<double, VectFull, Allocator1>& tau,
		   Vector<double, VectFull, Allocator2>& b,
		   LapackInfo& info = lapack_info);


  // MLTQ_FROMQR //
  /////////////////


  /////////////
  // SOLVEQR //


  /*** ColMajor ***/


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveQR(const Matrix<float, Prop0, ColMajor, Allocator0>& A,
	       const Vector<float, VectFull, Allocator1>& tau,
	       Vector<float, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveQR(const Matrix<double, Prop0, ColMajor, Allocator0>& A,
	       const Vector<double, VectFull, Allocator1>& tau,
	       Vector<double, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveQR(const Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
	       const Vector<complex<double>, VectFull, Allocator1>& tau,
	       Vector<complex<double>, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  /*** RowMajor ***/


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveQR(const Matrix<float, Prop0, RowMajor, Allocator0>& A,
	       const Vector<float, VectFull, Allocator1>& tau,
	       Vector<float, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveQR(const Matrix<double, Prop0, RowMajor, Allocator0>& A,
	       const Vector<double, VectFull, Allocator1>& tau,
	       Vector<double, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveQR(const Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A,
	       const Vector<complex<double>, VectFull, Allocator1>& tau,
	       Vector<complex<double>, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  // SOLVEQR //
  /////////////


  /////////////
  // SOLVELQ //


  /*** ColMajor ***/


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveLQ(const Matrix<float, Prop0, ColMajor, Allocator0>& A,
	       const Vector<float, VectFull, Allocator1>& tau,
	       Vector<float, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveLQ(const Matrix<double, Prop0, ColMajor, Allocator0>& A,
	       const Vector<double, VectFull, Allocator1>& tau,
	       Vector<double, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveLQ(const Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
	       const Vector<complex<double>, VectFull, Allocator1>& tau,
	       Vector<complex<double>, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  /*** RowMajor ***/


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveLQ(const Matrix<float, Prop0, RowMajor, Allocator0>& A,
	       const Vector<float, VectFull, Allocator1>& tau,
	       Vector<float, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveLQ(const Matrix<double, Prop0, RowMajor, Allocator0>& A,
	       const Vector<double, VectFull, Allocator1>& tau,
	       Vector<double, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  template <class Prop0, class Allocator0,
	    class Allocator1,class Allocator2>
  void SolveLQ(const Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A,
	       const Vector<complex<double>, VectFull, Allocator1>& tau,
	       Vector<complex<double>, VectFull, Allocator2>& b,
	       LapackInfo& info = lapack_info);


  // SOLVELQ //
  /////////////


} // end namespace

#define SELDON_FILE_LAPACK_LEAST_SQUARES_HXX
#endif

