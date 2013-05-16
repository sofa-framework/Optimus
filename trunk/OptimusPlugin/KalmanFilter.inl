/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: Stéphane Cotin                                                     *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ENGINE_KALMANFILTER_INL
#define SOFA_COMPONENT_ENGINE_KALMANFILTER_INL

#include <KalmanFilter.h>

// --------------------------- Example of Extended Kalman filter ------------------------//
/*
% A plane flights in a 2D space where the x axis is the distance traveled
% by the plane and y axis is its altitude.  This system can be represented
% by the fallowing equations:
% (This is just an example)
%
% xpp = F/m - bx/m * xp^2
% ypp = p/m * xp^2 - g
%
% where m is the plane's weight (1000 kg)
%       bx is the drag coefficient (0.35 N/m≤/s≤)
%       p is the lift force (3.92 N/m≤/s≤)
%       g is the gravitational acceleration (9.8 m/s≤)
%       F is the motor's thrust
%
% A station on the ground (at the origin) mesures the angle between the
% plane and the ground (x axis) and the distance between the plane and the station.
% These _Measures are based and the fallowing equations:
%
% theta = atan2(y,x)
% r = sqrt(x^2+y^2)
%
% The variance error matrix of the mesures is:
%
% R = [0.01^2  0
%      0       50^2]
%
% V = [1 0;
%      0 1];
%
% The variance error matrix of the plane's model is: WQW'
%
% Q = [0.01^2    0;
%      0         0.01^2];
%
% W = [0 0;
%      1 0;
%      0 0;
%      0 1];
%
*/

namespace sofa
{

namespace component
{

namespace engine
{

using namespace std;
using namespace Kalman;

template <class DataTypes>
KalmanFilter<DataTypes>::KalmanFilter()
: f_inputX ( initData (&f_inputX, "position", "current position") )
, f_inputV ( initData (&f_inputV, "velocity", "current velocity") )
, f_outputX( initData (&f_outputX, "output", "output array of 3D points") )
{
    _NTRY = 500;
    _nbStates = 4;
    _nbMeasures = 2;
}

template <class DataTypes>
KalmanFilter<DataTypes>::~KalmanFilter()
{
    _dataOutput.close();
    _dataInput.close();
}

template <class DataTypes>
void KalmanFilter<DataTypes>::init()
{
    std::string tmpStr;

    addInput(&f_inputX);
    addOutput(&f_outputX);
    setDirtyValue();

    Period = 0.2;
    Gravity = 9.8;
    Bfriction = 0.35;
    Portance = 3.92;
    Mass = 1000;

    Vector x(_nbStates);
    Matrix P0(_nbStates, _nbStates);
    P0.set(0,0, 100.0*100.0);   P0.set(0,1, 0.0);           P0.set(0,2, 0.0);           P0.set(0,3, 0.0);
    P0.set(1,0, 0.0);           P0.set(1,1, 10.0*10.0);     P0.set(1,2, 0.0);           P0.set(1,3, 0.0);
    P0.set(2,0, 0.0);           P0.set(2,1, 0.0);           P0.set(2,2, 25.0*25.0);     P0.set(1,3, 0.0);
    P0.set(3,0, 0.0);           P0.set(3,1, 10.0*10.0);     P0.set(3,2, 0.0);           P0.set(3,3, 10.0*10.0);

    _F.resize(_NTRY);
    _Measure.resize(_nbMeasures, _NTRY);

    _dataInput.open("/Users/stephane/Projects/newSofa/Sofa/examples/Sandbox/Cryoablation/Kalman/data.m",ifstream::in);
    _dataOutput.open("trajectory_udu_load.m", ofstream::out | ofstream::trunc);

    if (_dataInput.fail())
    {
        cout<<"Unable to open input file!"<<endl;
        return;
    }

    if (_dataOutput.fail())
    {
        cout<<"Unable to open output file!"<<endl;
        return;
    }

    cout<<"Loading inputs and measures from file <data.m>."<<endl;

    //Read the inputs vector. This vector have been generated by the Matlab script <generation.m>
    _dataInput >> tmpStr;
    _dataInput >> tmpStr;
    _dataInput >> _F;
    cout << "_F(" << _F.size() << ") = " << _F << endl;

    /*if (_dataInput.fail())
    {
        cout<<"IO error after reading _F!"<<endl;
        return;
    }*/

     //Read the _Measures matrix. This matrix have been generated by the Matlab script <generation.m>
    _dataInput >> tmpStr;
    cout << "tmpStr = " << tmpStr << endl;
    _dataInput >> tmpStr;
    cout << "tmpStr = " << tmpStr << endl;
    //_dataInput >> tmpStr;
    //_dataInput>>_Measure;

    if (_dataInput.fail())
    {
        cout<<"IO error!"<<endl;
        return;
    }

    unsigned i=1;

    //Initiale estimate
    std::cout << "angle: "<<_Measure[0][0]<< "radius: "<<_Measure[1][0]<<endl;

    this->x[0] = cos(_Measure[0][0])*_Measure[1][0];
    this->x[1] = 60;
    this->x[2] = sin(_Measure[0][0])*_Measure[1][0];
    this->x[3] = 0;

    this->initFilter(x, P0);
    cout << "xp(" << ":," << i<<") = " << this->getX()<<endl;
    _dataOutput<<"trajectory_udu(" << ":," << i <<") = " << this->getX()<<endl;

}

template <class DataTypes>
void KalmanFilter<DataTypes>::reinit()
{
    update();
}


// Simple uniform distribution of zero mean and unit variance
float uniform(void)
{
   return((((float)rand())/(RAND_MAX-1) - 0.5f)* 3.464101615138f);
}

// Simple approximation of normal dist. by the sum of uniform dist.
float normal()
{
  int n = 6;
  int i;
  float temp = 0.0;

  for(i = 0; i < n; i++)
    temp += uniform();
  temp /= sqrt((float)n);
  return temp;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeBaseA()
{
    std::cout << "size(A): " << this->A.rowSize() << "x" << this->A.colSize() << std::endl;

    this->A[0][0] = 1.0;
    // this->A[0][1] = Period - Period*Period*Bfriction/Mass*this->x[1];
    this->A[0][2] = 0.0;
    this->A[0][3] = 0.0;

    this->A[1][0] = 0.0;
    // this->A[1][1] = 1 - 2*Period*Bfriction/Mass*this->x[1];
    this->A[1][2] = 0.0;
    this->A[1][3] = 0.0;

    this->A[2][0] = 0.0;
    // this->A[2][1] = Period*Period*Portance/Mass*this->x[1];
    this->A[2][2] = 1.0;
    this->A[2][3] = Period;

    this->A[3][0] = 0.0;
    // this->A[3][1] = 2*Period*Portance/Mass*this->x[1];
    this->A[3][2] = 0.0;
    this->A[3][3] = 1.0;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeA()
{
    // this->A[0][0] = 1.0;
    this->A[0][1] = Period - Period*Period*Bfriction/Mass*this->x[1];
    // this->A[0][2] = 0.0;
    // this->A[0][3] = 0.0;

    // this->A[1][0] = 0.0;
    this->A[1][1] = 1 - 2*Period*Bfriction/Mass*this->x[1];
    // this->A[1][2] = 0.0;
    // this->A[1][3] = 0.0;

    // this->A[2][0] = 0.0;
    this->A[2][1] = Period*Period*Portance/Mass*this->x[1];
    // this->A[2][2] = 1.0;
    // this->A[2][3] = Period;

    // this->A[3][0] = 0.0;
    this->A[3][1] = 2*Period*Portance/Mass*this->x[1];
    // this->A[3][2] = 0.0;
    // this->A[3][3] = 1.0;
}

// -------------------------------------------------------------------
// W is the white noise that applies to the model (simulation)
// to account for uncertainties in the model
// -------------------------------------------------------------------
template <class DataTypes>
void KalmanFilter<DataTypes>::makeBaseW()
{
    std::cout << "size(W): " << this->W.rowSize() << "x" << this->W.colSize() << std::endl;

    this->W[0][0] = 0.0;
    this->W[0][1] = 0.0;
    this->W[1][0] = 1.0;
    this->W[1][1] = 0.0;
    this->W[2][0] = 0.0;
    this->W[2][1] = 0.0;
    this->W[3][0] = 0.0;
    this->W[3][1] = 1.0;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeBaseQ()
{
    this->Q[0][0] = 0.01*0.01;
    this->Q[0][1] = 0.01*0.01/10.0;
    this->Q[1][0] = 0.01*0.01/10.0;
    this->Q[1][1] = 0.01*0.01;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeBaseH()
{
    // this->H[0][0] = -x[2]/(x[0]*x[0]+x[2]*x[2]);
    this->H[0][1] = 0.0;
    // this->H[0][2] = x[0]/(x[0]*x[0]+x[2]*x[2]);
    this->H[0][3] = 0.0;

    // this->H[1][0] = x[0]/sqrt(x[0]*x[0]+x[2]*x[2]);
    this->H[1][1] = 0.0;
    // this->H[1][2] = x[2]/sqrt(x[0]*x[0]+x[2]*x[2]);
    this->H[1][3] = 0.0;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeH()
{
    this->H[0][0] = -this->x[2]/(this->x[0]*this->x[0]+this->x[2]*this->x[2]);
    // this->H[0][1] = 0.0;
    this->H[0][2] = this->x[0]/(this->x[0]*this->x[0]+this->x[2]*this->x[2]);
    // this->H[0][3] = 0.0;

    this->H[1][0] = this->x[0]/sqrt(this->x[0]*this->x[0]+this->x[2]*this->x[2]);
    // this->H[1][1] = 0.0;
    this->H[1][2] = this->x[2]/sqrt(this->x[0]*this->x[0]+this->x[2]*this->x[2]);
    // this->H[1][3] = 0.0;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeBaseV()
{
    this->V[0][0] = 1.0;
    this->V[1][1] = 1.0;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeBaseR()
{
    this->R[0][0] = 0.01*0.01;
    this->R[1][1] = 50*50;
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeProcess()
{
    Vector x_(this->x.size());

    x_[0] = this->x[0] + this->x[1]*Period + (Period*Period)/2*(this->u[0]/Mass - Bfriction/Mass*this->x[1]*this->x[1]);
    x_[1] = this->x[1] + (this->u[0]/Mass - Bfriction/Mass*this->x[1]*this->x[1])*Period;
    x_[2] = this->x[2] + this->x[3]*Period + (Period*Period)/2*(Portance/Mass*this->x[1]*this->x[1]-Gravity);
    x_[3] = this->x[3] + (Portance/Mass*this->x[1]*this->x[1]-Gravity)*Period;
    this->x.swap(x_);
}

template <class DataTypes>
void KalmanFilter<DataTypes>::makeMeasure()
{
    this->z[0] = atan2(this->x[2], this->x[0]);
    this->z[1] = sqrt(this->x[0]*this->x[0]+this->x[2]*this->x[2]);
}


template <class DataTypes>
void KalmanFilter<DataTypes>::update()
{
    cleanDirty();

    helper::ReadAccessor<Data <VecVec1d> > input = f_inputX;
    std::cout << "KalmanFilter :: input size : " << f_inputX.getValue().size() << std::endl;

    Vector z(_nbMeasures);

    for (unsigned i = 1; i < _NTRY; ++i)
	{
		// filter
        for(unsigned j = 0; j < _nbMeasures; j++)
            z[j] = _Measure[j][i];

        Vector u(1, _F[i]);

        this->step(u, z);

        cout << "xp(" << ":," << i<<") = " << this->getX()<<endl;
        _dataOutput<<"trajectory_udu(" << ":," << i<<") = " << this->getX()<<endl;
	}

}

} // namespace engine

} // namespace component

} // namespace sofa

#endif
