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
#ifndef ShowCatheter_INL
#define ShowCatheter_INL

#include "genericComponents/ShowCatheter.h"

#include <sofa/core/visual/VisualParams.h>


namespace sofa
{
namespace component
{
namespace engine
{

template <class DataTypes>
ShowCatheter<DataTypes>::ShowCatheter()
    : _positions( initData(&_positions, "position", "positions to draw the spheres") )
    , _indices( initData(&_indices, "indices", "indices of the position vector to be shown") )
    , _draw( initData(&_draw, true, "draw", "draw the spheres") )
    , _radius( initData(&_radius, "radius", "radius of the spheres") )
    , _showIndicesSize( initData(&_showIndicesSize, float(0.0), "showIndicesScale", "size of indices") )
    , _color( initData(&_color, "color", "color of the spheres") )
    , _indexColor( initData(&_indexColor, Vec4f(0.0,0.0,0.0,1.0), "indexColor", "color of the indices") )
{    
}

template <class DataTypes>
ShowCatheter<DataTypes>::~ShowCatheter()
{
}

template<class DataTypes>
void ShowCatheter<DataTypes>::draw(const core::visual::VisualParams* vparams) {
    if (vparams->displayFlags().getShowVisualModels()) {
        helper::ReadAccessor< Data< VecCoord > > pos = _positions;
        helper::vector<int> ind = _indices.getValue();
        std::vector<std::vector<sofa::defaulttype::Vec3d>> predpoints;

        // if indices are empty, take all the nodes
        if (ind.size() == 0) {
            ind.resize(pos.size());
            for (size_t i = 0; i < ind.size(); i++)
                ind[i] = int(i);
        } else {
            for (size_t i = 0; i < ind.size(); i++) {
                if (ind[i] >= pos.size()) {
                    std::cerr << "ERROR SPHERES: index " << ind[i] << " out of scope of coord vector " << pos.size() << std::endl;
                    return;
                }
            }
        }

        // Check topological modifications
        const int npoints = ind.size();
        const float rad = _radius.getValue();

        std::vector<Vector3> points(npoints);
        std::vector<float> radii(npoints);
        for (int i=0; i<npoints; i++)
        {
            points[i] = pos[ind[i]];
            radii[i] = rad;

        }

        vparams->drawTool()->setPolygonMode(0,vparams->displayFlags().getShowWireFrame());
        vparams->drawTool()->setLightingEnabled(true); //Enable lightning
        vparams->drawTool()->drawSpheres(points, radii, _color.getValue());
        vparams->drawTool()->setPolygonMode(0,false);
        vparams->drawTool()->drawLineStrip(points, 3.0, _color.getValue());


        if (_showIndicesSize.getValue() > 0) {
            const Vec4f indCol = _indexColor.getValue();
            float scale = (float)((vparams->sceneBBox().maxBBox() - vparams->sceneBBox().minBBox()).norm() * _showIndicesSize.getValue());

            for (size_t i = 0; i < npoints; i++) {
                const Vector3 &p = pos[ind[i]];
                std::string text = std::to_string(ind[i]);
                vparams->drawTool()->draw3DText(p, scale, indCol, text.c_str());
            }
//            vparams->drawTool()->draw3DText_Indices(points, scale, indCol);
        }

    }
}

//template <class DataTypes>
//void ShowCatheter<DataTypes>::init()
//{
//
//}
//
//template <class DataTypes>
//void ShowCatheter<DataTypes>::reinit()
//{
//}

} // engine
} // component
} // sofa

#endif // ShowCatheter_INL
