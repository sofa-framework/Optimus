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
// This file is part of kfilter.
// kfilter is a C++ variable-dimension extended kalman filter library.
//
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef EKFILTER_IMPL_INL
#define EKFILTER_IMPL_INL

#include "initOptimusPlugin.h"
#include "ekfilter.hpp"

//! \file
//! \brief Contains the implementation of the \c EKFilter base template class.

//! \internal 
//! Flag : \a n has changed
#define KALMAN_N_MODIFIED    1

//! \internal
//! Flag : \a nu has changed
#define KALMAN_NU_MODIFIED  (1<<1)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NW_MODIFIED  (1<<2)

//! \internal
//! Flag : \a m has changed
#define KALMAN_M_MODIFIED   (1<<3)

//! \internal
//! Flag : \a nv has changed
#define KALMAN_NV_MODIFIED  (1<<4)

//! \internal
//! Flag : \a P has changed
#define KALMAN_P_MODIFIED   (1<<5)

//! \internal
//! Mask : used to reset dimension flags
#define KALMAN_LOWMASK      ((1<<8) - 1)

//! \internal
//! Flag : \a A has changed
#define KALMAN_A_MODIFIED   (1<<8)

//! \internal
//! Flag : \a W has changed
#define KALMAN_W_MODIFIED   (1<<9)

//! \internal
//! Flag : \a Q has changed
#define KALMAN_Q_MODIFIED   (1<<10)

//! \internal
//! Mask : used to reset time update matrix flags
#define KALMAN_MIDMASK      ( ((1<<4) - 1) << 8 )

//! \internal
//! Flag : \a H has changed
#define KALMAN_H_MODIFIED   (1<<12)

//! \internal
//! Flag : \a V has changed
#define KALMAN_V_MODIFIED   (1<<13)

//! \internal
//! Flag : \a R has changed
#define KALMAN_R_MODIFIED   (1<<14)

//! \internal
//! Mask : used to reset measure update matrix flags
#define KALMAN_HIGHMASK     ( ((1<<4) - 1) << 12 )

namespace Kalman {

    template<typename T, bool OQ, bool OVR>
    EKFilter<T,OQ, OVR>::EKFilter()
        : flags(0) {}

  template<typename T, bool OQ, bool OVR>
  EKFilter<T,OQ, OVR>::EKFilter(unsigned int n_, unsigned int nu_, unsigned int nw_, unsigned int m_, unsigned int nv_)
    : flags(0) {
    setDim(n_, nu_, nw_, m_, nv_);
  }

  template<typename T, bool OQ, bool OVR>
  EKFilter<T,OQ, OVR>::~EKFilter() {}

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::setDim(unsigned int n_, unsigned int nu_,
                                              unsigned int nw_, unsigned int m_,
                                              unsigned int nv_) {
    setSizeX(n_);
    setSizeU(nu_);
    setSizeW(nw_);
    setSizeZ(m_);
    setSizeV(nv_);
  }

  template<typename T, bool OQ, bool OVR>
  unsigned int EKFilter<T,OQ, OVR>::getSizeX() const {
    return n;
  }

  template<typename T, bool OQ, bool OVR>
  unsigned int EKFilter<T,OQ, OVR>::getSizeU() const {
    return nu;
  }

  template<typename T, bool OQ, bool OVR>
  unsigned int EKFilter<T,OQ, OVR>::getSizeW() const {
    return nw;
  }

  template<typename T, bool OQ, bool OVR>
  unsigned int EKFilter<T,OQ, OVR>::getSizeZ() const {
    return m;
  }

  template<typename T, bool OQ, bool OVR>
  unsigned int EKFilter<T,OQ, OVR>::getSizeV() const {
    return nv;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::setSizeX(unsigned int n_) {

    // verify : n_ > 0

    if (n_ != n) {
      flags |= KALMAN_N_MODIFIED;
      n = n_;
    }
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::setSizeU(unsigned int nu_) {
    if (nu_ != nu) {
      flags |= KALMAN_NU_MODIFIED;
      nu = nu_;
    }
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::setSizeW(unsigned int nw_) {
    if (nw_ != nw) {
      flags |= KALMAN_NW_MODIFIED;
      nw = nw_;
    }
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::setSizeZ(unsigned int m_) {
    if (m_ != m) {
      flags |= KALMAN_M_MODIFIED;
      m = m_;
    }
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::setSizeV(unsigned int nv_) {
    if (nv_ != nv) {
      flags |= KALMAN_NV_MODIFIED;
      nv = nv_;
    }
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::initFilter(Vector& x_, Matrix& P_) {

    // verify : (x_.size() == n && P_.rowSize() == n && P_.ncol() == n)

    // swap _P and P_
      swap(_P, P_);
    // end of swap

    // swap x and x_
    x.swap(x_);

    flags |= KALMAN_P_MODIFIED;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::step(Vector& u_, const Vector& z_) {
    timeUpdateStep(u_);
    measureUpdateStep(z_);
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::timeUpdateStep(Vector& u_) {

    // verif : u_.size() == nu
    unsigned int i, j, k;

    sizeUpdate();
    u.swap(u_);
    
    makeCommonProcess();
    makeAImpl();
    makeWImpl();
    makeQImpl();
    makeProcess();

    if (!OQ) {

      if (flags & KALMAN_Q_MODIFIED) {

        Q_ = Q;
        factor(Q_);
        upperInvert(Q_);

      }

      swap(Q, Q_);
      
      // W_ = W*U   n.nw = n.nw * nw.nw

      if (flags & ( KALMAN_W_MODIFIED | KALMAN_Q_MODIFIED ) ) {

        for (i = 0; i < n + 0; ++i) {
    
          for (j = 0; j < nw + 0; ++j) {
      
            W_[i][j] = W[i][j];
            for (k = 0; k < j; ++k)
              W_[i][j] += W[i][k]*Q[j][k];
      
          }
    
        }
  
      }
      swap(W, W_);
    }

    timeUpdate();

    if (!OQ) {
      swap(Q, Q_);
      swap(W, W_);
    }

    u.swap(u_);
    flags &= ~KALMAN_MIDMASK;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::measureUpdateStep(const Vector& z_) {

    // verif : z_.size() == m
    unsigned int i, j, k;

    sizeUpdate();

    if (m == 0) {
      return;
    }
    
    makeCommonMeasure();
    makeHImpl();
    makeVImpl();
    makeRImpl();    
    makeMeasure();
    
    // verif : nv != 0

    for (i = 0; i < m; ++i)
        dz[i] = z_[i] - z[i];

    makeDZ();

    if (OVR) {

      // verif : m == nv

      if (flags & ( KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) {

        for (i = 0; i < m; ++i)
            R_[i][i] = V[i][i]*V[i][i]*R[i][i];

      }

    } else {


      if (flags & ( KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) { // calculate R_

        _x.resize(nv);
      
        // R_ = V*R*V'
        for (i = 0; i < m; ++i) {

          // _x = row i of V*R = (V*R)(i,:)
          for (j = 0; j < nv; ++j) {

              _x[j] = T(0.0);
            for (k = 0; k < nv; ++k)
                _x[j] += V[i][k]*R[k][j];

          }

          // R_(i,:) = (V*R*V')(i,:) = (V*R)(i,:) * V'
          for (j = 0; j < m; ++j) {

            R_[i][j] = T(0.0);
            for (k = 0; k < nv; ++k)
              R_[i][j] += _x[k]*V[j][k];

          }

        }

        // R_ = U*D*U'
        // diag(R_) = D, upper(R_) = U, lower(R_) = junk
        factor(R_);

        // lower(R_) = (inv(U))'
        upperInvert(R_);

      }

      if (flags & ( KALMAN_H_MODIFIED | KALMAN_V_MODIFIED | KALMAN_R_MODIFIED ) ) { // calculate H_

        // H_ = inv(U)*H    m.n = m.m * m.n
        for (i = 0; i < m; ++i) {

          for (j = 0; j < n; ++j) {

            H_[i][j] = H[i][j];
            for (k = i + 1; k < m; ++k)
              H_[i][j] += R_[k][i]*H[k][j];

          }

        }

      }

      swap(H, H_);

      // _x = inv(U)*dz    m.1 = m.m * m.1
      _x.resize(m);

      for (i = 0; i < m; ++i) {

        _x[i] = dz[i];
        for (k = i + 1; k < m; ++k)
          _x[i] += R_[k][i]*dz[k];

      }

      dz.swap(_x);

    }
    
    _x.resize(n); // dx : innovation
    _x.clear();

    for (i = 0; i < m; ++i) {

      for (j = 0; j < n; ++j)
        a[j] = H[i][j];

      measureUpdate(dz[i], R_[i][i]);

    }
    for (i = 0; i < n; ++i)
      x[i] += _x[i];

    if (!OVR) {
      swap(H, H_);
    }

    flags &= ~KALMAN_HIGHMASK;
  }

  template<typename T, bool OQ, bool OVR>
  const typename EKFilter<T,OQ, OVR>::Vector& EKFilter<T,OQ, OVR>::predict(Vector& u_) {
    
    // verif : u_.size() == nu

    sizeUpdate();
    u.swap(u_);   
    _x = x;
    
    makeCommonProcess();
    makeProcess();
    
    x.swap(_x);
    u.swap(u_);
    return _x;
  }

  template<typename T, bool OQ, bool OVR>
  const typename EKFilter<T,OQ, OVR>::Vector& EKFilter<T,OQ, OVR>::simulate() {
    
    sizeUpdate();
    _x = z;
    
    makeCommonMeasure();
    makeMeasure();
    
    z.swap(_x);
    return _x;
  }

  template<typename T, bool OQ, bool OVR>
  const typename EKFilter<T,OQ, OVR>::Vector& EKFilter<T,OQ, OVR>::getX() const {
    return x;
  }

  template<typename T, bool OQ, bool OVR>
  const typename EKFilter<T,OQ, OVR>::Matrix& EKFilter<T,OQ, OVR>::calculateP() const {

    if (!(flags & KALMAN_P_MODIFIED)) {

      _P.resize(n, n);         // keep this resize
    
      for (unsigned int i = 0; i < n; ++i) {

        _P[i][i] = U[i][i];

        for (unsigned int j = i + 1; j < n; ++j) {

          _P[i][j]  = U[i][j]*U[j][j];
          _P[i][i] += U[i][j]*_P[i][j];

          for (unsigned int k = j + 1; k < n; ++k) {
            _P[i][j] += U[i][k]*U(j,k)*U(k,k);
          }

          _P[j][i] = _P[i][j];

        }

      }

    }

    return _P;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::NoModification() {
    modified_ = false;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseA() {
    NoModification();
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseW() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseQ() {
    NoModification();
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseH() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseV() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseR() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeCommonProcess() {}

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeCommonMeasure() {}

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeA() {
    NoModification();
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeW() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeQ() {
    NoModification();
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeH() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeV() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeR() {
    NoModification();
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeDZ() {}

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::sizeUpdate() {
    
    if (!flags) {
      return;
    }

    if (flags & KALMAN_N_MODIFIED) {
      A.resize(n, n);
      makeBaseAImpl();
    }

    if (flags & (KALMAN_N_MODIFIED | KALMAN_NW_MODIFIED) ) {
      nn = n + nw;
      a.resize(nn);
      v.resize(nn);
      d.resize(nn);
      if (!OQ)
        W_.resize(n, nw);
      W.resize(n, nw);
      makeBaseWImpl();
    }

    // KALMAN_N_MODIFIED imply KALMAN_P_MODIFIED
    // => KALMAN_N_MODIFIED must not be set OR KALMAN_P_MODIFIED must be set
    // => NOT  KALMAN_N_MODIFIED  OR  KALMAN_P_MODIFIED  must be set
    // verif : (flags ^ KALMAN_N_MODIFIED) & 
    //              (KALMAN_N_MODIFIED | KALMAN_P_MODIFIED)

    if (flags & KALMAN_P_MODIFIED) { 
      // this covers the case of KALMAN_N_MODIFIED = true also

      // We have a new matrix P : let's factorize it and store it in U
      // First, resize U and copy P in its left part
      U.resize(n, nn);
      for (unsigned int i = 0; i < n; ++i)
        for (unsigned int j = 0; j < n; ++j)
            U[i][j] = _P[i][j];
      
      // Factorize
      factor(U);

    } else if (flags & KALMAN_NW_MODIFIED) {
      // KALMAN_N_MODIFIED is necessarily false, else KALMAN_P_MODIFIED
      // would have been true

      // Let's just copy U in temporary matrix _P of the right size,
      // then swap the matrices
      _P.resize(n, nn);
      for (unsigned int i = 0; i < n; ++i)
        for (unsigned int j = i; j < n; ++j)
          _P[i][j] = U[i][j];
      swap(U, _P);
    }

    if (flags & KALMAN_NW_MODIFIED) {
      if (!OQ)
        Q_.resize(nw, nw);
      Q.resize(nw, nw);
      makeBaseQImpl();
    }

    if (m != 0) {

      if (flags & (KALMAN_N_MODIFIED | KALMAN_M_MODIFIED) ) {
        if (!OVR)
          H_.resize(m, n);
        H.resize(m, n);
        makeBaseHImpl();
      }

      if (flags & (KALMAN_M_MODIFIED | KALMAN_NV_MODIFIED) ) {
        V.resize(m, nv);
        makeBaseVImpl();
      }

      if (flags & KALMAN_NV_MODIFIED) {
        R.resize(nv, nv);
        makeBaseRImpl();
      }

      if (flags & KALMAN_M_MODIFIED) {
        R_.resize(m, m);
        z.resize(m);
        dz.resize(m);
      }

    }
    
    flags &= ~KALMAN_LOWMASK;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::factor(Matrix& P_) {

    // ne pas vérifier que P_.ncol() == P_.rowSize(), comme ça, même si
    // rowSize() < colSize(), on peut factoriser la sous-matrice carrée de P
    // Utile pour factoriser U

    T alpha, beta;
    unsigned int i, j, k, N = P_.rowSize();
    for(j = N - 1; j > 0; --j) {
      alpha = T(1.0)/P_[j][j];
      for(k = 0; k < j; ++k) {
          beta = P_[k][j];
        P_[k][j] = alpha*beta;
        for(i = 0; i <= k; ++i)
            P_[i][k] -= beta*P_[i][j];
      }
    }
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::upperInvert(Matrix& P_) {

    T val;
    unsigned int i, j, k, N = P_.rowSize();
    for (i = N - 2; i != (unsigned int)(0-1); --i) { // ???
      for (k = i + 1; k < N; ++k) {

        val = P_[i][k];
        for (j = i + 1; j <= k - 1; ++j)
          val += P_[i][j]*P_[k][j];
        P_[k][i] = -val;

      }
    }

  }

  // U    u     U-D covariance matrix (n,nn)
  // A    phi   transition matrix (F) (n,n)
  // W    g     process noise matrix (G) (n,nw)
  // Q    q     process noise variance vector (nw) Q = diag(q)
  // a, v, d temporary vectors
  // U is updated
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::timeUpdate() {

    unsigned int i, j, k;
    T sigma, dinv;
  
    // U = phi * U
    // d = diag(U)
    // 
    // This algo could be faster
    // if phi is known to be diagonal
    // It could be almost zapped if phi=I
    for(j = n - 1; j > 0; --j) {
      for(i = 0; i <= j; ++i)
        d[i] = U[i][j];
      for(i = 0; i < n; ++i) {
        U[i][j] = A[i][j];
        for(k = 0; k < j; ++k)
          U[i][j] += A[i][k]*d[k];
      }
    }

    d[0] = U[0][0];
    for(j = 0; j < n; ++j)
      U[j][0] = A[j][0];

    // d(n+1:nn) = q 
    // U(:,n+1:nn) = G 
    for(i = 0; i < nw; ++i) {
      d[i+n] = Q[i][i];
      for(j = 0; j < n; ++j)
        U[j][i+n] = W[j][i];
    }

    // Gram-Schmidt
    // Too hard to simplify
    for(j = n - 1; j != (unsigned int)(0-1); --j) { // ???
      sigma = T(0.0);
      for(k = 0; k < nn; ++k) {
        v[k] = U[j][k];
        a[k] = d[k]*v[k];
        sigma += v[k]*a[k];
      }
      U[j][j] = sigma;
      if(j == 0 || sigma == T(0.0)) continue;
      dinv = T(1.0)/sigma;
      for(k = 0; k < j; ++k) {
        sigma = T(0.0);
        for(i = 0; i < nn; ++i)
          sigma += U[k][i]*a[i];
        sigma *= dinv;
        for(i = 0; i < nn; ++i)
          U[k][i] -= sigma*v[i];
        U[j][k] = sigma;
      }
    }

    // U = transpose(U)
    for(j = 0 + 1; j < n; ++j)
      for(i = 0; i < j; ++i)
        U[i][j] = U[j][i];
  }

  // x     a priori estimate vector (n)
  // U     a priori U-D covariance matrix (n,nn)
  // dz    measurement diff (z - ax) (scalar)
  // a     measurement coefficients vector (n) (a row of A, which is H)
  //          a is destroyed
  // r     measurement variance
  // d is a temporary vector
  // x and U are updated
  // a is destroyed
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::measureUpdate(T dz, T r) {

    unsigned int i, j, k;
    T alpha, gamma, beta, lambda;

    // dz = dz - Hdx
    for (j = 0; j < n; ++j)
      dz -= a[j]*_x[j];
    
    // d = D * transpose(U) * a
    // a =     transpose(U) * a
    //
    // This algo could be faster
    // if A is known to be diagonal or I
    for(j = n - 1; j > 0; --j) {
      for(k = 0; k < j; ++k)
        a[j] += U[k][j]*a[k];
      d[j] = U[j][j]*a[j];
    }
    d[0] = U[0][0]*a[0];

    // UDU
    // Too hard to simplify
    alpha = r+d[0]*a[0];
    gamma = T(1.0)/alpha;
    U[0][0] = r*gamma*U[0][0];
    for(j = 0 + 1; j < n; ++j) {
      beta = alpha;
      alpha += d[j]*a[j];
      lambda = -a[j]*gamma;
      gamma = T(1.0)/alpha;
      U[j][j] *= beta*gamma;
      for(i = 0; i < j; ++i) {
        beta = U[i][j];
        U[i][j] = beta+d[i]*lambda;
        d[i] += d[j]*beta;
      }
    }
  
    // dx = dx + K(dz - Hdx)
    dz *= gamma;
    for(j = 0; j < n; ++j)
      _x[j] += d[j]*dz;
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseAImpl() {
    modified_ = true;
    makeBaseA();
    if (modified_)
      flags |= KALMAN_A_MODIFIED;
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseWImpl() {
    modified_ = true;
    makeBaseW();
    if (modified_)
      flags |= KALMAN_W_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseQImpl() {
    modified_ = true;
    makeBaseQ();
    if (modified_)
      flags |= KALMAN_Q_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseHImpl() {
    modified_ = true;
    makeBaseH();
    if (modified_)
      flags |= KALMAN_H_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseVImpl() {
    modified_ = true;
    makeBaseV();
    if (modified_)
      flags |= KALMAN_V_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeBaseRImpl() {
    modified_ = true;
    makeBaseR();
    if (modified_)
      flags |= KALMAN_R_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeAImpl() {
    modified_ = true;
    makeA();
    if (modified_)
      flags |= KALMAN_A_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeWImpl() {
    modified_ = true;
    makeW();
    if (modified_)
      flags |= KALMAN_W_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeQImpl() {
    modified_ = true;
    makeQ();
    if (modified_)
      flags |= KALMAN_Q_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeHImpl() {
    modified_ = true;
    makeH();
    if (modified_)
      flags |= KALMAN_H_MODIFIED;    
  }
  
  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeVImpl() {
    modified_ = true;
    makeV();
    if (modified_)
      flags |= KALMAN_V_MODIFIED;    
  }

  template<typename T, bool OQ, bool OVR>
  void EKFilter<T,OQ, OVR>::makeRImpl() {
    modified_ = true;
    makeR();
    if (modified_)
      flags |= KALMAN_R_MODIFIED;    
  }

}

#endif
