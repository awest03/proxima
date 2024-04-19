/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PROXIMA_IMPL_HPP_INCLUDED
#define PROXIMA_IMPL_HPP_INCLUDED

#include <proxima/proxima_decl.hpp>

#include <cmath>
#include <list>

namespace proxima
{

inline void MovementDirection::fromVector(float x, float y)
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

inline float MovementDirection::getAngle() const
{
    float x = dir & (uint8_t)Direction::X ? 1.0f : 0.0f;
    if (dir & (uint8_t)Direction::InvertX) x = -x;
    float y = dir & (uint8_t)Direction::Y ? 1.0f : 0.0f;
    if (dir & (uint8_t)Direction::InvertY) y = -y;

    return std::atan2(y, x);
}

inline bool MovementDirection::hasMagnitude() const
{
    return (dir & (uint8_t)Direction::X) || (dir & (uint8_t)Direction::Y);
}

template <typename C, typename I, typename S>
inline std::tuple<bool, uint16_t> getDirectPathLow(const Grid<uint8_t, C, I, S> *costField, C x0, C y0, C x1, C y1)
{
    uint16_t totalCost = 0;

    S dx = S(x1) - S(x0);
    S dy = S(y1) - S(y0);

    S yi = 1;
    if (dy < 0)
    {
        yi = -1;
        dy = -dy;
    }
    S D = (2 * dy) - dx;
    S y = y0;

    for (S x = x0; x < S(x1); x++)
    {
        uint8_t cost = (*costField)(x, y);
        if (cost == 255) return { false, totalCost };
        totalCost += cost;
        if (D > 0)
        {
            y += yi;
            D += (2 * (dy - dx));
        }
        else
        {
            D += 2*dy;
        }
    }

    return { true, totalCost };
}

template <typename C, typename I, typename S>
inline std::tuple<bool, uint16_t> getDirectPathHigh(const Grid<uint8_t, C, I, S> *costField, C x0, C y0, C x1, C y1)
{
    uint16_t totalCost = 0;

    S dx = S(x1) - S(x0);
    S dy = S(y1) - S(y0);

    S xi = 1;
    if (dx < 0)
    {
        xi = -1;
        dx = -dx;
    }
    S D = (2 * dx) - dy;
    S x = x0;

    for (S y = y0; y < S(y1); y++)
    {
        uint8_t cost = (*costField)(x, y);
        if (cost == 255) return { false, totalCost };
        totalCost += cost;
        if (D > 0)
        {
            x += xi;
            D += (2 * (dx - dy));
        }
        else
        {
            D += 2*dx;
        }
    }

    return { true, totalCost };
}

// Based on Bresenham's line alogorithm
template <typename C, typename I, typename S>
inline std::tuple<bool, I> GetDirectPath(const Grid<uint8_t, C, I, S> *costField, const I a, const I b)
{
    auto [x0, y0] = costField->getCoordinate(a);
    auto [x1, y1] = costField->getCoordinate(b);

    if (std::abs(S(y1) - S(y0)) < std::abs(S(x1) - S(x0)))
    {
        if (x0 > x1)
            return getDirectPathLow(costField, x1, y1, x0, y0);
        else
            return getDirectPathLow(costField, x0, y0, x1, y1);
    }
    else
    {
        if (y0 > y1)
            return getDirectPathHigh(costField, x1, y1, x0, y0);
        else
            return getDirectPathHigh(costField, x0, y0, x1, y1);
    }
}

// Based on algorithm described by https://web.archive.org/web/20190725152730/http://aigamedev.com/open/tutorial/clearance-based-pathfinding/
template <typename C, typename I, typename S>
uint8_t GetCellClearance(const Grid<uint8_t, C, I, S> *costField, const C x, const C y, const uint8_t maxClearance)
{
    for (uint8_t i = 0; ; i++)
    {
        C diagx = x + i;
        C diagy = y + i;

        if (diagx >= costField->width || diagy >= costField->height) return i - 1;

        if ((*costField)(diagx, diagy) == 255) return i;
        for (uint8_t j = 1; j < i; j++)
        {
            // Check vertically
            if ((*costField)(diagx, diagy - j) == 255) return i;
            // Check horizontally
            if ((*costField)(diagx - j, diagy) == 255) return i;
        }

        if (maxClearance != 0 && i == maxClearance) return i;
    }
}

template <typename C, typename I, typename S>
void GenerateIntegrationField(const Grid<uint8_t, C, I, S> *cost, const I target, Grid<uint16_t, C, I, S> *result)
{
    result->fill(65535);

    std::list<I> openList;
    (*result)[target] = 0;
    openList.push_back(target);

    while (!openList.empty())
    {
        // Get next node in open list
        I id = openList.front();
        openList.pop_front();

        auto [x, y] = cost->getCoordinate(id);

        // Get neighbours
        std::vector<I> neighbours = cost->getDirectNeighbours(x, y);

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

template <typename C, typename I, typename S>
inline void CombineIntegrationFields(const Grid<uint16_t, C, I, S> *a, const Grid<uint16_t, C, I, S> *b, Grid<uint16_t, C, I, S> *c)
{   
    for (I i = 0; i < c->area(); i++)
    {
        (*c)[i] = std::min((*a)[i], (*b)[i]);
    }
}

template <typename C, typename I, typename S>
inline I GetBestNeighbour(const Grid<uint16_t, C, I, S> *intField, const C x, const C y)
{
    uint16_t best = (*intField)(x, y);
    I bestId = intField->getIndex(x, y);

    bool top = false, bottom = false, left = false, right = false;

    // First check direct neighbours
    std::vector<I> direct = intField->getDirectNeighbours(x, y);
    for (uint16_t &n : direct)
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
    std::vector<I> diagonal = intField->getDiagonalNeighbours(x, y);
    for (I &n : diagonal)
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

template <typename C, typename I, typename S>
MovementDirection GetBestDirection(const Grid<uint16_t, C, I, S> *intField, const C x, const C y)
{
    I bestId = GetBestNeighbour(intField, x, y);
    auto [bx, by] = intField->getCoordinate(bestId);
    float vx = float(bx) - float(x);
    float vy = float(by) - float(y);
    MovementDirection md;
    md.fromVector(vx, vy);
    return md;
}

}

#endif