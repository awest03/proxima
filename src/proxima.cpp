/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <proxima/proxima.hpp>

#include <cmath>
#include <list>

using namespace proxima;

void MovementDirection::fromVector(float x, float y)
{
    dir = 0;
    if (std::abs(x) > 0.0f)
    {
        dir |= (uint8_t)Direction::X;
        if (x < 0.0f)
            dir |= (uint8_t)Direction::InvertX;
    }
    if (std::abs(y) > 0.0f)
    {
        dir |= (uint8_t)Direction::Y;
        if (y < 0.0f)
            dir |= (uint8_t)Direction::InvertY;
    }
}

float MovementDirection::getAngle() const
{
    float x = dir & (uint8_t)Direction::X ? 1.0f : 0.0f;
    if (dir & (uint8_t)Direction::InvertX) x = -x;
    float y = dir & (uint8_t)Direction::Y ? 1.0f : 0.0f;
    if (dir & (uint8_t)Direction::InvertY) y = -y;

    return std::atan2(y, x);
}

bool MovementDirection::hasMagnitude() const
{
    return (dir & (uint8_t)Direction::X) || (dir & (uint8_t)Direction::Y);
}

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

uint32_t proxima::GetBestNeighbour(const Grid<uint16_t> *intField, const uint32_t x, const uint32_t y)
{
    uint16_t best = (*intField)(x, y);
    uint32_t bestId = intField->getIndex(x, y);

    bool top = false, bottom = false, left = false, right = false;

    // First check direct neighbours
    std::vector<uint32_t> direct = intField->getDirectNeighbours(x, y);
    for (uint32_t &n : direct)
    {
        if ((*intField)[n] < best)
        {
            best = (*intField)[n];
            bestId = n;
        }
        else if ((*intField)[n] == 65535)
        {
            auto [nx, ny] = intField->getCoordinate(n);
            if (nx < x) left = true;
            else if (nx > x) right = true;
            else if (ny > y) top = true;
            else if (ny < y) bottom = true;
        }
    }

    // Now check diagonal neighbours
    std::vector<uint32_t> diagonal = intField->getDiagonalNeighbours(x, y);
    for (uint32_t &n : diagonal)
    {
        if ((*intField)[n] < best)
        {
            // More complicated: Can we actually reach this cell?
            auto [nx, ny] = intField->getCoordinate(n);
            // Left...
            if (nx < x && !left)
                goto check;
            else if (nx > x && !right) // Right...
                goto check;
            
            // Cell is unreachable
            continue;
            
            // Check if the cell is top or bottom
            check:
            // Top
            if (ny > y && top)
                continue;
            else if (ny < y && bottom) // Bottom
                continue;

            best = (*intField)[n];
            bestId = n;
        }
    }

    return bestId;
}

MovementDirection proxima::GetBestDirection(const Grid<uint16_t> *intField, const uint32_t x, const uint32_t y)
{
    uint32_t bestId = GetBestNeighbour(intField, x, y);
    auto [bx, by] = intField->getCoordinate(bestId);
    float vx = float(bx) - float(x);
    float vy = float(by) - float(y);
    MovementDirection md;
    md.fromVector(vx, vy);
    return md;
}