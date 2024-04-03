/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <proxima/proxima.hpp>

#include <cmath>
#include <list>

using namespace proxima;

void proxima::GenerateIntegrationField(const Grid<uint8_t> *cost, const uint32_t target, Grid<uint16_t> *result)
{
    result->fill(65535);

    std::list<uint32_t> openList;
    (*result)[target] = 0;
    openList.push_back(target);

    while (!openList.empty())
    {
        // Get next node in open list
        uint32_t id = openList.front();
        openList.pop_front();

        auto [x, y] = cost->getCoordinate(id);

        // Get neighbours
        std::vector<uint32_t> neighbours = cost->getDirectNeighbours(x, y);

        for (uint32_t i = 0; i < neighbours.size(); i++)
        {
            if ((*cost)[neighbours[i]] == 255)
                continue;

            uint32_t endCost = (*result)[id] + (*cost)[neighbours[i]];
            if (endCost < (*result)[neighbours[i]])
            {
                openList.push_back(neighbours[i]);
                (*result)[neighbours[i]] = (uint16_t)endCost;
            }
        }
    }
}

void proxima::CombineIntegrationFields(const Grid<uint16_t> *a, const Grid<uint16_t> *b, Grid<uint16_t> *c)
{   
    for (uint32_t i = 0; i < c->area(); i++)
    {
        (*c)[i] = std::min((*a)[i], (*b)[i]);
    }
}

void proxima::GenerateVectorField(const Grid<uint16_t> *intField, Grid<float> *result)
{
    result->fill(0.0f);

    for (uint32_t i = 0; i < intField->area(); i++)
    {
        auto [x, y] = intField->getCoordinate(i);
        std::vector<uint32_t> neighbours = intField->getAllNeighbours(x, y);

        uint16_t best = (*intField)[i];
        uint32_t bestId = i;
        for (auto &n : neighbours)
        {
            if ((*intField)[n] < best)
            {
                best = (*intField)[n];
                bestId = n;
            }
        }
        auto [bx, by] = intField->getCoordinate(bestId);
        float vx = float(bx) - float(x);
        float vy = float(by) - float(y);
        (*result)[i] = std::atan2(vy, vx);
    }
}