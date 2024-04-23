/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PROXIMA_MESH_INCLUDED
#define PROXIMA_MESH_INCLUDED

#include <vector>

namespace proxima
{

struct Polygon
{
    void addVertex(float x, float y);
    size_t vertices() const;

    std::vector<float> x;
    std::vector<float> y;
};

}

#endif