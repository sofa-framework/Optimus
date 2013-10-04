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


#ifndef SELDON_FILE_BLAS_1_HXX

/*
  Functions included in this file:

  xROTG   (GenRot)
  xROTMG  (GenModifRot)
  xROT    (ApplyRot)
  xROTM   (ApplyModifRot)
  xSWAP   (Swap)
  xSCAL   (Mlt)
  xCOPY   (Copy)
  xAXPY   (Add)
  xDOT    (DotProd)
  xDOTU   (DotProd)
  SDSDOT  (ScaledDotProd)
  xDOTC   (DotProdConj)
  xASUM   (Norm1)
  xNRM2   (Norm2)
  IxAMAX  (GetMaxAbsIndex)
*/

extern "C"
{
#include "cblas.h"
}

namespace Seldon
{


  ////////////
  // GenRot //


  void GenRot(float& a, float& b, float& c, float& d);

  void GenRot(double& a, double& b, double& c, double& d);

  // GenRot //
  ////////////



  /////////////////
  // GenModifRot //


  void GenModifRot(float& d1, float& d2,
		   float& x1, const float& y1,
		   float* param);

  void GenModifRot(double& d1, double& d2,
		   double& x1, const double& y1,
		   double* param);


  // GenModifRot //
  /////////////////



  //////////////
  // ApplyRot //


  template <class Allocator>
  void ApplyRot(Vector<float, VectFull, Allocator>& X,
		Vector<float, VectFull, Allocator>& Y,
		const float c, const float s);

  template <class Allocator>
  void ApplyRot(Vector<double, VectFull, Allocator>& X,
		Vector<double, VectFull, Allocator>& Y,
		const double c, const double s);


  // ApplyRot //
  //////////////



  ///////////////////
  // ApplyModifRot //


  template <class Allocator>
  void ApplyModifRot(Vector<float, VectFull, Allocator>& X,
		     Vector<float, VectFull, Allocator>& Y,
		     const float* param);

  template <class Allocator>
  void ApplyModifRot(Vector<double, VectFull, Allocator>& X,
		     Vector<double, VectFull, Allocator>& Y,
		     const double* param);


  // ApplyModifRot //
  ///////////////////



  //////////
  // Swap //


  template <class Allocator>
  void Swap(Vector<float, VectFull, Allocator>& X,
	    Vector<float, VectFull, Allocator>& Y);

  template <class Allocator>
  void Swap(Vector<double, VectFull, Allocator>& X,
	    Vector<double, VectFull, Allocator>& Y);

  template <class Allocator>
  void Swap(Vector<complex<float>, VectFull, Allocator>& X,
	    Vector<complex<float>, VectFull, Allocator>& Y);

  template <class Allocator>
  void Swap(Vector<complex<double>, VectFull, Allocator>& X,
	    Vector<complex<double>, VectFull, Allocator>& Y);


  // Swap //
  //////////



  /////////
  // Mlt //


  template <class Allocator>
  void Mlt(const float alpha,
	   Vector<float, VectFull, Allocator>& X);

  template <class Allocator>
  void Mlt(const double alpha,
	   Vector<double, VectFull, Allocator>& X);

  template <class Allocator>
  void Mlt(const float alpha,
	   Vector<complex<float>, VectFull, Allocator>& X);

  template <class Allocator>
  void Mlt(const double alpha,
	   Vector<complex<double>, VectFull, Allocator>& X);

  template <class Allocator>
  void Mlt(const complex<float> alpha,
	   Vector<complex<float>, VectFull, Allocator>& X);

  template <class Allocator>
  void Mlt(const complex<double> alpha,
	   Vector<complex<double>, VectFull, Allocator>& X);


  // Mlt //
  /////////



  //////////
  // Copy //


  template <class Allocator0, class Allocator1>
  void Copy(const Vector<float, VectFull, Allocator0>& X,
	    Vector<float, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  void Copy(const Vector<double, VectFull, Allocator0>& X,
	    Vector<double, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  void Copy(const Vector<complex<float>, VectFull, Allocator0>& X,
	    Vector<complex<float>, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  void Copy(const Vector<complex<double>, VectFull, Allocator0>& X,
	    Vector<complex<double>, VectFull, Allocator1>& Y);


  // Copy //
  //////////



  /////////
  // Add //


  template <class Allocator0, class Allocator1>
  void Add(const float alpha,
	   const Vector<float, VectFull, Allocator0>& X,
	   Vector<float, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  void Add(const double alpha,
	   const Vector<double, VectFull, Allocator0>& X,
	   Vector<double, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  void Add(const complex<float> alpha,
	   const Vector<complex<float>, VectFull, Allocator0>& X,
	   Vector<complex<float>, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  void Add(const complex<double> alpha,
	   const Vector<complex<double>, VectFull, Allocator0>& X,
	   Vector<complex<double>, VectFull, Allocator1>& Y);


  // Add //
  /////////



  /////////////
  // DotProd //


  template <class Allocator0, class Allocator1>
  float DotProd(const Vector<float, VectFull, Allocator0>& X,
		const Vector<float, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  double DotProd(const Vector<double, VectFull, Allocator0>& X,
		 const Vector<double, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  complex<float>
  DotProd(const Vector<complex<float>, VectFull, Allocator0>& X,
	  const Vector<complex<float>, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  complex<double>
  DotProd(const Vector<complex<double>, VectFull, Allocator0>& X,
	  const Vector<complex<double>, VectFull, Allocator1>& Y);


  // DotProd //
  /////////////



  ///////////////////
  // SCALEDDOTPROD //


  template <class Allocator0, class Allocator1>
  float ScaledDotProd(const float alpha,
		      const Vector<float, VectFull, Allocator0>& X,
		      const Vector<float, VectFull, Allocator1>& Y);


  // SCALEDDOTPROD //
  ///////////////////



  /////////////////
  // DotProjConj //


  template <class Allocator0, class Allocator1>
  complex<float>
  DotProdConj(const Vector<complex<float>, VectFull, Allocator0>& X,
	      const Vector<complex<float>, VectFull, Allocator1>& Y);

  template <class Allocator0, class Allocator1>
  complex<double>
  DotProdConj(const Vector<complex<double>, VectFull, Allocator0>& X,
	      const Vector<complex<double>, VectFull, Allocator1>& Y);


  // DotProdConj //
  /////////////////



  ///////////
  // Norm1 //


  template <class Allocator>
  float Norm1(const Vector<float, VectFull, Allocator>& X);

  template <class Allocator>
  double Norm1(const Vector<double, VectFull, Allocator>& X);

  template <class Allocator>
  float Norm1(const Vector<complex<float>, VectFull, Allocator>& X);

  template <class Allocator>
  double Norm1(const Vector<complex<double>, VectFull, Allocator>& X);


  // Norm1 //
  ///////////



  ///////////
  // Norm2 //


  template <class Allocator>
  float Norm2(const Vector<float, VectFull, Allocator>& X);

  template <class Allocator>
  double Norm2(const Vector<double, VectFull, Allocator>& X);

  template <class Allocator>
  float Norm2(const Vector<complex<float>, VectFull, Allocator>& X);

  template <class Allocator>
  double Norm2(const Vector<complex<double>, VectFull, Allocator>& X);


  // Norm2 //
  ///////////



  ////////////////////
  // GetMaxAbsIndex //


  template <class Allocator>
  size_t GetMaxAbsIndex(const Vector<float, VectFull, Allocator>& X);

  template <class Allocator>
  size_t GetMaxAbsIndex(const Vector<double, VectFull, Allocator>& X);

  template <class Allocator>
  size_t GetMaxAbsIndex(const Vector<complex<float>, VectFull, Allocator>& X);

  template <class Allocator>
  size_t
  GetMaxAbsIndex(const Vector<complex<double>, VectFull, Allocator>& X);


  // GetMaxAbsIndex //
  ////////////////////


} // namespace Seldon.

#define SELDON_FILE_BLAS_1_HXX
#endif
