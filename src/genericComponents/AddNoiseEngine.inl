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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "AddNoiseEngine.h"


namespace sofa
{
namespace component
{
namespace engine
{

template <class DataTypes>
AddNoiseEngine<DataTypes>::AddNoiseEngine()
    : _inputPositions( initData(&_inputPositions, "inputPositions", "input positions which will be subjected to noise perturbation"))
    , _outputPositionsWithNoise( initData(&_outputPositionsWithNoise, "outputPositions", "perturbed positions"))
    , _noisePrincipalDirection( initData(& _noisePrincipalDirection, Coord(0.0, 0.0, 1.0), "noisePrincipalDirection", "direction in which the principal noise is applied"))
    , _noiseType( initData(&_noiseType, std::string("gaussian"), "noiseType", "type of noise, options: gaussian (default)"))
    , _noiseMean( initData(&_noiseMean, double(0.0), "noiseMean", "mean of the noise (exp. value)"))
    , _noisePrincipalVariance( initData(&_noisePrincipalVariance, double(1.0), "noisePrincipalVariance", "variance applied in the principal direction"))
    , _noiseOrthogonalVariance( initData(&_noiseOrthogonalVariance, double(1.0), "noiseOrthogonalVariance", "variance applied in the orthogonal directions"))
{
    addInput(&_inputPositions);
    addOutput(&_outputPositionsWithNoise);
    setDirtyValue();
}

template <class DataTypes>
void AddNoiseEngine<DataTypes>::init() {
//    std::cout << "init start" << std::endl;
    dir = _noisePrincipalDirection.getValue();
    dir.normalize();
    
    //generate the orthogonal directions
    Coord test;
    orth1.set(0.0,1.0,0.0);   // NV
    orth2.set(0.0,0.0,1.0);   // BV
       
//    test=cross(dir,orth2);
////    std::cout << "O1 = " << orth1 << std::endl;
    
    if (test.norm() < 1e-5) {
        orth2=cross(dir, orth1);
        orth2.normalize();
        orth1=cross(orth2,dir);
        orth1.normalize();
    } else {
        orth1=cross(orth2,dir);
        orth1.normalize();
        orth2=cross(orth1,dir);
        orth2.normalize();
    }

//    std::cout << "dir = " << dir << std::endl;
//    std::cout << "O1 = " << orth1 << std::endl;
//    std::cout << "O2 = " << orth2 << std::endl;

    helper::ReadAccessor< Data< VecCoord > > inputPos = _inputPositions;
    helper::WriteAccessor< Data< VecCoord > > outputPos = _outputPositionsWithNoise;



}

template<class DataTypes>
void AddNoiseEngine<DataTypes>::update() {
    helper::ReadAccessor< Data< VecCoord > > inputPos = _inputPositions;
    helper::WriteAccessor< Data< VecCoord > > outputPos = _outputPositionsWithNoise;

    outputPos.resize(inputPos.size());

//    std::cout <<"Resize " << inputPos.size() << " -> " << outputPos.size() <<  std::endl;


    double mean = _noiseMean.getValue();
    double varPrinc = _noisePrincipalVariance.getValue();
    double varOrtho = _noiseOrthogonalVariance.getValue();

    //std::cout << this->getName() << ": applying principal noise (var = " << varPrinc << ") in direction : " << dir << std::endl;
    //std::cout << this->getName() << ": applying orthogonal noise (var = " << varOrtho << ") in directions : " << orth1 << " and " << orth2 << std::endl;

    for (size_t i = 0; i < inputPos.size(); i++) {
        Coord pert = dir * getRandomGauss(mean,varPrinc) +  orth1 * getRandomGauss(mean,varOrtho) + orth2 * getRandomGauss(mean,varOrtho);
//        std::cout << "Pert = " << pert << std::endl;
        outputPos[i] = inputPos[i] + pert;
//        std::cout << "Pos[" << i << "]= " << outputPos[i] << std::endl;

    }
}


template<class DataTypes>
double  AddNoiseEngine<DataTypes>::getRandomGauss(const double mean, const double sigma) {

      typedef std::chrono::high_resolution_clock myclock;
      static unsigned int seed = myclock::now().time_since_epoch().count();
      static std::default_random_engine generator(seed);
      static std::normal_distribution<double> distribution(0.0,1.0);
      return distribution(generator)*sigma + mean;

}

template <class DataTypes>
void AddNoiseEngine<DataTypes>::handleEvent(core::objectmodel::Event *event) {
    if (dynamic_cast<sofa::simulation::AnimateEndEvent *>(event))
    {
        update();
    }
}


} // engine
} // component
} // sofa

