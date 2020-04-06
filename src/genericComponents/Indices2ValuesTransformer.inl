/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef INDECES2VALUESTRANSFORMER_INL_
#define INDECES2VALUESTRANSFORMER_INL_

#include "Indices2ValuesTransformer.h"

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/system/FileRepository.h>

namespace sofa
{

namespace component
{

namespace engine
{

using namespace sofa;
using namespace sofa::core::topology;

template <class DataTypes>
Indices2ValuesTransformer<DataTypes>::Indices2ValuesTransformer()
    : f_inputValues(initData(&f_inputValues, "inputValues", "Already existing values (can be empty) "))
    , f_indices(initData(&f_indices, "indices", "Indices to map value on "))
    , f_values1(initData(&f_values1, "values1", "First set of values to map indices on "))
    , f_values2(initData(&f_values2, "values2", "Second set ot values to map indices on "))
    , f_outputValues(initData(&f_outputValues, "outputValues", "New map between indices and values"))
    , p_defaultValue(initData(&p_defaultValue, Real(0.0), "defaultValue", "Default value for indices without any value"))
    , d_transformation(initData(&d_transformation,std::string("ENu2Lame"),"transformation","identifier of transformation"))
{
}

template <class DataTypes>
void Indices2ValuesTransformer<DataTypes>::init()
{
    addInput(&f_inputValues);
    addInput(&f_indices);
    addInput(&f_values1);
    addInput(&f_values2);

    addOutput(&f_outputValues);

    setDirtyValue();
}

template <class DataTypes>
void Indices2ValuesTransformer<DataTypes>::reinit()
{
    update();
}

template <class DataTypes>
void Indices2ValuesTransformer<DataTypes>::doUpdate()
{
    cleanDirty();

    helper::ReadAccessor< Data< helper::vector<Real> > > inputValues = f_inputValues;
    helper::ReadAccessor< Data< helper::vector<Real> > > indices = f_indices;
    helper::ReadAccessor< Data< helper::vector<Real> > > values1 = f_values1;
    helper::ReadAccessor< Data< helper::vector<Real> > > values2 = f_values2;

    helper::WriteAccessor< Data< helper::vector<Real> > > outputValues = f_outputValues;

    const Real& defaultValue = p_defaultValue.getValue();

    if (values1.size() != values2.size()) {
        msg_error() << " Size of the value vectors must be the same!";
        return;
    }

    if (values1.size() != indices.size()) {
        msg_error() << " Numer of indices must be same as the size of the value vector!";
        return;
    }

    //copy existing values
    outputValues.clear();
    if (d_transformation.getValue() == "ENu2MuLambda") {
        outputValues.resize(2*inputValues.size());

        //add new value
        for(unsigned int i=0 ; i<inputValues.size() ; i++) {
            bool found = false;
            for (size_t j = 0; j < indices.size(); j++) {
                if (inputValues[i] == indices[j]) {
                    Real E=values1[j];
                    Real nu = values2[j];

                    outputValues[2*i] = E/(2+2*nu);
                    outputValues[2*i+1] = (E*nu)/((1+nu)*(1-2*nu));
                    found = true;
                    break;
                }
            }
            if (!found) {
                msg_warning() << "Value for index " << inputValues[i] << " not defined, using default value " << defaultValue << "!";
                outputValues[2*i] = defaultValue;
                outputValues[2*i+1] = defaultValue;
            }
        }
    } else {
        msg_error() << " Unknown transformation. ";
    }    

}


} // namespace engine

} // namespace component

} // namespace sofa

#endif //INDECES2VALUESTRANSFORMER_INL_
