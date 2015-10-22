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


#ifndef SELDON_FILE_BLAS_2_HXX

/*
  Functions included in this file:

  xTRMV   (Mlt)
  xTPMV   (Mlt)
  xGEMV   (MltAdd)
  xHEMV   (MltAdd)
  xHPMV   (MltAdd)
  xSYMV   (MltAdd)
  xSPMV   (MltAdd)
  xGER    (Rank1Update)
  xGERU   (Rank1Update)
  xGERC   (Rank1Update)
  xSPR    (Rank1Update)
  xHPR    (Rank1Update)
  xSPR2   (Rank2Update)
  xHPR2   (Rank2Update)
  xTRSV   (Solve)
  xTPSV   (Solve)
*/

extern "C"
{
#include "cblas.h"
}

namespace Seldon
{


  /////////
  // MLT //


  /*** ColUpTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, ColUpTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, ColUpTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, ColUpTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, ColUpTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);

  /*** ColUpTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, ColUpTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, ColUpTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, ColUpTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, ColUpTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, ColLoTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, ColLoTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, ColLoTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, ColLoTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, ColLoTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, ColLoTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, ColLoTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, ColLoTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, RowUpTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, RowUpTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, RowUpTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, RowUpTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, RowUpTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, RowUpTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, RowUpTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, RowUpTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, RowLoTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, RowLoTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, RowLoTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, RowLoTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, RowLoTriang, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, RowLoTriang, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, RowLoTriang, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, RowLoTriang, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColUpTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, ColUpTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, ColUpTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, ColUpTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, ColUpTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColUpTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, ColUpTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, ColUpTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, ColUpTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, ColUpTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, ColLoTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, ColLoTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, ColLoTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, ColLoTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, ColLoTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, ColLoTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, ColLoTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, ColLoTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, RowUpTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, RowUpTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, RowUpTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, RowUpTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, RowUpTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, RowUpTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, RowUpTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, RowUpTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<float, Prop0, RowLoTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const Matrix<double, Prop0, RowLoTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<float>, Prop0, RowLoTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const Matrix<complex<double>, Prop0, RowLoTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<float, Prop0, RowLoTriangPacked, Allocator0>& A,
	   Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Mlt(const SeldonTranspose& TransA,
	   const SeldonDiag& DiagA,
	   const Matrix<double, Prop0, RowLoTriangPacked, Allocator0>& A,
	   Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<float>, Prop0, RowLoTriangPacked, Allocator0>& A,
      Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Mlt(const SeldonTranspose& TransA,
      const SeldonDiag& DiagA,
      const Matrix<complex<double>, Prop0, RowLoTriangPacked, Allocator0>& A,
      Vector<complex<double>, VectFull, Allocator1>& X);


  // MLT //
  /////////



  ////////////
  // MLTADD //


  // General //

  /*** ColMajor and NoTrans ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const Matrix<float, Prop0, ColMajor, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const Matrix<double, Prop0, ColMajor, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const Matrix<complex<float>, Prop0, ColMajor, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** ColMajor and TransA ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<float, Prop0, ColMajor, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<double, Prop0, ColMajor, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<complex<float>, Prop0, ColMajor, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** RowMajor and NoTrans ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const Matrix<float, Prop0, RowMajor, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const Matrix<double, Prop0, RowMajor, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const Matrix<complex<float>, Prop0, RowMajor, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** RowMajor and TransA ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<float, Prop0, RowMajor, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<double, Prop0, RowMajor, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<complex<float>, Prop0, RowMajor, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const SeldonTranspose& TransA,
	      const Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  // Hermitian //

  /*** ColHerm and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const Matrix<complex<float>, Prop0, ColHerm, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const Matrix<complex<double>, Prop0, ColHerm, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** ColHerm and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<float>, Prop0, ColHerm, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<double>, Prop0, ColHerm, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** RowHerm and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const Matrix<complex<float>, Prop0, RowHerm, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const Matrix<complex<double>, Prop0, RowHerm, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** RowHerm and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<float>, Prop0, RowHerm, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<double>, Prop0, RowHerm, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  // HermPacked //

  /*** ColHermPacked and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const Matrix<complex<float>, Prop0,
	      ColHermPacked, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const Matrix<complex<double>, Prop0,
	      ColHermPacked, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** ColHermPacked and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<float>, Prop0,
	      ColHermPacked, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<double>, Prop0,
	      ColHermPacked, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** RowHermPacked and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const Matrix<complex<float>, Prop0,
	      RowHermPacked, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const Matrix<complex<double>, Prop0,
	      RowHermPacked, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  /*** RowHermPacked and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<float> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<float>, Prop0,
	      RowHermPacked, Allocator0>& A,
	      const Vector<complex<float>, VectFull, Allocator1>& X,
	      const complex<float> beta,
	      Vector<complex<float>, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const complex<double> alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<complex<double>, Prop0,
	      RowHermPacked, Allocator0>& A,
	      const Vector<complex<double>, VectFull, Allocator1>& X,
	      const complex<double> beta,
	      Vector<complex<double>, VectFull, Allocator2>& Y);


  // Symmetric //

  /*** ColSym and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const Matrix<float, Prop0, ColSym, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const Matrix<double, Prop0, ColSym, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  /*** ColSym and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<float, Prop0, ColSym, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<double, Prop0, ColSym, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  /*** RowSym and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const Matrix<float, Prop0, RowSym, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const Matrix<double, Prop0, RowSym, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  /*** RowSym and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<float, Prop0, RowSym, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<double, Prop0, RowSym, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  // SymPacked //

  /*** ColSymPacked and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const Matrix<float, Prop0, ColSymPacked, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const Matrix<double, Prop0, ColSymPacked, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  /*** ColSymPacked and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<float, Prop0, ColSymPacked, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<double, Prop0, ColSymPacked, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  /*** RowSymPacked and Upper ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const Matrix<float, Prop0, RowSymPacked, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const Matrix<double, Prop0, RowSymPacked, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  /*** RowSymPacked and Uplo ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const float alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<float, Prop0, RowSymPacked, Allocator0>& A,
	      const Vector<float, VectFull, Allocator1>& X,
	      const float beta,
	      Vector<float, VectFull, Allocator2>& Y);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void MltAdd(const double alpha,
	      const SeldonUplo& Uplo,
	      const Matrix<double, Prop0, RowSymPacked, Allocator0>& A,
	      const Vector<double, VectFull, Allocator1>& X,
	      const double beta,
	      Vector<double, VectFull, Allocator2>& Y);


  // MLTADD //
  ////////////



  /////////////////
  // RANK1UPDATE //


  /*** ColMajor ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const float alpha,
		   const Vector<float, VectFull, Allocator1>& X,
		   const Vector<float, VectFull, Allocator2>& Y,
		   Matrix<float, Prop0, ColMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const double alpha,
		   const Vector<double, VectFull, Allocator1>& X,
		   const Vector<double, VectFull, Allocator2>& Y,
		   Matrix<double, Prop0, ColMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<float> alpha,
		   const Vector<complex<float>, VectFull, Allocator1>& X,
		   const Vector<complex<float>, VectFull, Allocator2>& Y,
		   Matrix<complex<float>, Prop0, ColMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<double> alpha,
		   const Vector<complex<double>, VectFull, Allocator1>& X,
		   const Vector<complex<double>, VectFull, Allocator2>& Y,
		   Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A);


  /*** ColMajor and ConjY ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<float> alpha,
		   const Vector<complex<float>, VectFull, Allocator1>& X,
		   const SeldonConjugate& ConjY,
		   const Vector<complex<float>, VectFull, Allocator2>& Y,
		   Matrix<complex<float>, Prop0, ColMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<double> alpha,
		   const Vector<complex<double>, VectFull, Allocator1>& X,
		   const SeldonConjugate& ConjY,
		   const Vector<complex<double>, VectFull, Allocator2>& Y,
		   Matrix<complex<double>, Prop0, ColMajor, Allocator0>& A);


  /*** RowMajor ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const float alpha,
		   const Vector<float, VectFull, Allocator1>& X,
		   const Vector<float, VectFull, Allocator2>& Y,
		   Matrix<float, Prop0, RowMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const double alpha,
		   const Vector<double, VectFull, Allocator1>& X,
		   const Vector<double, VectFull, Allocator2>& Y,
		   Matrix<double, Prop0, RowMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<float> alpha,
		   const Vector<complex<float>, VectFull, Allocator1>& X,
		   const Vector<complex<float>, VectFull, Allocator2>& Y,
		   Matrix<complex<float>, Prop0, RowMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<double> alpha,
		   const Vector<complex<double>, VectFull, Allocator1>& X,
		   const Vector<complex<double>, VectFull, Allocator2>& Y,
		   Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A);


  /*** RowMajor and ConjY ***/


  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<float> alpha,
		   const Vector<complex<float>, VectFull, Allocator1>& X,
		   const SeldonConjugate& ConjY,
		   const Vector<complex<float>, VectFull, Allocator2>& Y,
		   Matrix<complex<float>, Prop0, RowMajor, Allocator0>& A);

  template <class Prop0, class Allocator0,
	    class Allocator1, class Allocator2>
  void Rank1Update(const complex<double> alpha,
		   const Vector<complex<double>, VectFull, Allocator1>& X,
		   const SeldonConjugate& ConjY,
		   const Vector<complex<double>, VectFull, Allocator2>& Y,
		   Matrix<complex<double>, Prop0, RowMajor, Allocator0>& A);


  /*** ColSymPacked and Upper ***/


  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   Matrix<float, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   Matrix<double, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const float alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      Matrix<complex<float>, Prop1, ColHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const double alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      Matrix<complex<double>, Prop1, ColHermPacked, Allocator1>& A);


  /*** ColSymPacked and Uplo ***/


  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   const SeldonUplo& Uplo,
		   Matrix<float, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   const SeldonUplo& Uplo,
		   Matrix<double, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const float alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      const SeldonUplo& Uplo,
	      Matrix<complex<float>, Prop1, ColHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const double alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      const SeldonUplo& Uplo,
	      Matrix<complex<double>, Prop1, ColHermPacked, Allocator1>& A);


  /*** RowSymPacked and Upper ***/


  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   Matrix<float, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   Matrix<double, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const float alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      Matrix<complex<float>, Prop1, RowHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const double alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      Matrix<complex<double>, Prop1, RowHermPacked, Allocator1>& A);


  /*** RowSymPacked and Uplo ***/


  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   const SeldonUplo& Uplo,
		   Matrix<float, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void Rank1Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   const SeldonUplo& Uplo,
		   Matrix<double, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const float alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      const SeldonUplo& Uplo,
	      Matrix<complex<float>, Prop1, RowHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1>
  void
  Rank1Update(const double alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      const SeldonUplo& Uplo,
	      Matrix<complex<double>, Prop1, RowHermPacked, Allocator1>& A);


  // RANK1UPDATE //
  /////////////////



  /////////////////
  // RANK2UPDATE //


  /*** ColSymPacked and Upper ***/


  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   const Vector<float, VectFull, Allocator2>& Y,
		   Matrix<float, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   const Vector<double, VectFull, Allocator2>& Y,
		   Matrix<double, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<float> alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      const Vector<complex<float>, VectFull, Allocator2>& Y,
	      Matrix<complex<float>, Prop1, ColHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<double> alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      const Vector<complex<double>, VectFull, Allocator2>& Y,
	      Matrix<complex<double>, Prop1, ColHermPacked, Allocator1>& A);


  /*** ColSymPacked and Uplo ***/


  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   const Vector<float, VectFull, Allocator2>& Y,
		   const SeldonUplo& Uplo,
		   Matrix<float, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   const Vector<double, VectFull, Allocator2>& Y,
		   const SeldonUplo& Uplo,
		   Matrix<double, Prop1, ColSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<float> alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      const Vector<complex<float>, VectFull, Allocator2>& Y,
	      const SeldonUplo& Uplo,
	      Matrix<complex<float>, Prop1, ColHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<double> alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      const Vector<complex<double>, VectFull, Allocator2>& Y,
	      const SeldonUplo& Uplo,
	      Matrix<complex<double>, Prop1, ColHermPacked, Allocator1>& A);


  /*** RowSymPacked and Upper ***/


  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   const Vector<float, VectFull, Allocator2>& Y,
		   Matrix<float, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   const Vector<double, VectFull, Allocator2>& Y,
		   Matrix<double, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<float> alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      const Vector<complex<float>, VectFull, Allocator2>& Y,
	      Matrix<complex<float>, Prop1, RowHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<double> alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      const Vector<complex<double>, VectFull, Allocator2>& Y,
	      Matrix<complex<double>, Prop1, RowHermPacked, Allocator1>& A);


  /*** RowSymPacked and Uplo ***/


  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const float alpha,
		   const Vector<float, VectFull, Allocator0>& X,
		   const Vector<float, VectFull, Allocator2>& Y,
		   const SeldonUplo& Uplo,
		   Matrix<float, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void Rank2Update(const double alpha,
		   const Vector<double, VectFull, Allocator0>& X,
		   const Vector<double, VectFull, Allocator2>& Y,
		   const SeldonUplo& Uplo,
		   Matrix<double, Prop1, RowSymPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<float> alpha,
	      const Vector<complex<float>, VectFull, Allocator0>& X,
	      const Vector<complex<float>, VectFull, Allocator2>& Y,
	      const SeldonUplo& Uplo,
	      Matrix<complex<float>, Prop1, RowHermPacked, Allocator1>& A);

  template <class Allocator0,
	    class Prop1, class Allocator1,
	    class Allocator2>
  void
  Rank2Update(const complex<double> alpha,
	      const Vector<complex<double>, VectFull, Allocator0>& X,
	      const Vector<complex<double>, VectFull, Allocator2>& Y,
	      const SeldonUplo& Uplo,
	      Matrix<complex<double>, Prop1, RowHermPacked, Allocator1>& A);


  // RANK2UPDATE //
  /////////////////



  ///////////
  // SOLVE //


  /*** ColUpTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, ColUpTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, ColUpTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, ColUpTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	ColUpTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColUpTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, ColUpTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, ColUpTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, ColUpTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	ColUpTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, ColLoTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, ColLoTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, ColLoTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	ColLoTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, ColLoTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, ColLoTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, ColLoTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	ColLoTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, RowUpTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, RowUpTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, RowUpTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	RowUpTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, RowUpTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, RowUpTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, RowUpTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	RowUpTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriang, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, RowLoTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, RowLoTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, RowLoTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	RowLoTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriang ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, RowLoTriang, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, RowLoTriang, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, RowLoTriang, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	RowLoTriang, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColUpTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, ColUpTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, ColUpTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, ColUpTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	ColUpTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColUpTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, ColUpTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, ColUpTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, ColUpTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	ColUpTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, ColLoTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, ColLoTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, ColLoTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	ColLoTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** ColLoTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, ColLoTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, ColLoTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, ColLoTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	ColLoTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, RowUpTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, RowUpTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, RowUpTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	RowUpTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowUpTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, RowUpTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, RowUpTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, RowUpTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	RowUpTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriangPacked, NoTrans and NonUnit ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<float, Prop0, RowLoTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const Matrix<double, Prop0, RowLoTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<float>, Prop0, RowLoTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const Matrix<complex<double>, Prop0,
	RowLoTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  /*** RowLoTriangPacked ***/


  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<float, Prop0, RowLoTriangPacked, Allocator0>& A,
	     Vector<float, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void Solve(const SeldonTranspose& TransA,
	     const SeldonDiag& DiagA,
	     const Matrix<double, Prop0, RowLoTriangPacked, Allocator0>& A,
	     Vector<double, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<float>, Prop0, RowLoTriangPacked, Allocator0>& A,
	Vector<complex<float>, VectFull, Allocator1>& X);

  template <class Prop0, class Allocator0,
	    class Allocator1>
  void
  Solve(const SeldonTranspose& TransA,
	const SeldonDiag& DiagA,
	const Matrix<complex<double>, Prop0,
	RowLoTriangPacked, Allocator0>& A,
	Vector<complex<double>, VectFull, Allocator1>& X);


  // SOLVE //
  ///////////


} // namespace Seldon.

#define SELDON_FILE_BLAS_2_HXX
#endif
