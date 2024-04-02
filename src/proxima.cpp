/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <proxima/proxima.hpp>

#include <cmath>
#include <list>

namespace proxima
{

Grid<uint16_t> GenerateIntegrationField(const Grid<uint8_t> &costField, const std::vector<uint32_t> &targets)
{
    Grid<uint16_t> integration(costField.width(), costField.height());
    integration.fill(65535);

    std::list<uint32_t> openList;
    integration[targets[0]] = 0;
    openList.push_back(targets[0]);

    while (!openList.empty())
    {
        // Get next node in open list
        uint32_t id = openList.front();
        openList.pop_front();

        auto [x, y] = costField.getCoordinate(id);

        // Get neighbours
        std::vector<uint32_t> neighbours = costField.getDirectNeighbours(x, y);

        for (uint32_t i = 0; i < neighbours.size(); i++)
        {
            if (costField[neighbours[i]] == 255)
                continue;

            uint32_t endCost = integration[id] + costField[neighbours[i]];
            if (endCost < integration[neighbours[i]])
            {
                openList.push_back(neighbours[i]);
                integration[neighbours[i]] = (uint16_t)endCost;
            }
        }
    }

    return integration;
}

Grid<float> GenerateVectorField(const Grid<uint16_t> &integrationField)
{
    Grid<float> direction(integrationField.width(), integrationField.height());
    direction.fill(0.0f);

    for (uint32_t i = 0; i < integrationField.area(); i++)
    {
        auto [x, y] = integrationField.getCoordinate(i);
        std::vector<uint32_t> neighbours = integrationField.getAllNeighbours(x, y);

        uint16_t best = integrationField[i];
        uint32_t bestId = i;
        for (auto &n : neighbours)
        {
            if (integrationField[n] < best)
            {
                best = integrationField[n];
                bestId = n;
            }
        }
        auto [bx, by] = integrationField.getCoordinate(bestId);
        float vx = float(bx) - float(x);
        float vy = float(by) - float(y);
        direction[i] = std::atan2(vy, vx);
    }

    return direction;
}

Grid<float> GenerateVectorField(const Grid<uint8_t> &costField, const std::vector<uint32_t> &targets)
{
    Grid<uint16_t> integration = GenerateIntegrationField(costField, targets);
    return GenerateVectorField(integration);
}

} // namespace proxima