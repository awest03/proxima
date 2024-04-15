/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PROXIMA_HPP_INCLUDED
#define PROXIMA_HPP_INCLUDED

#include <proxima/grid.hpp>

namespace proxima
{

/**
 * @brief Since there are only 8 possible directions we don't need a float
 * 
 */
enum class Direction : uint8_t
{
    X = 1 << 0,
    InvertX = 1 << 1,
    Y = 1 << 2,
    InvertY = 1 << 3,
};

/**
 * @brief Wrapper class to make directions easier to work with
 * 
 */
struct MovementDirection
{
    void fromVector(float x, float y);

    float getAngle() const;
    bool hasMagnitude() const;

    uint8_t dir;
};

/**
 * @brief Determines whether there is a direct line of sight from a to b and calculates its cost
 * 
 * @param costField The cost field to consider (cannot be NULL)
 * @param a the id of the first cell
 * @param b the id of the second cell
 * @return std::tuple<bool, uint16_t> whether there is a direct path and if so it's cost
 */
std::tuple<bool, uint16_t> GetDirectPath(const Grid<uint8_t> *costField, const uint16_t a, const uint16_t b);

/**
 * @brief Gets the clearance of a cell, i.e. how large of an object can pass through tiles down and right
 * 
 * @param costField The cost field to consider (cannot be NULL)
 * @param x x coordinate of the cell to consider
 * @param y y coordinate of the cell to consider
 * @param maxClearance maximum clearance to consider (0 for unlimited, not recommended for performance)
 * @return the clearance of the cell
 */
uint8_t GetCellClearance(const Grid<uint8_t> *costField, const uint8_t x, const uint8_t y, const uint8_t maxClearance);

/**
 * @brief Generates the integration field due to target (the cell index)
 * 
 * @param costField The cost field to consider (cannot be NULL)
 * @param target The target/goal of the pathfinding
 * @param result Grid in which to store the result (cannot be NULL). Must have same width and height as the cost field
 */
void GenerateIntegrationField(const Grid<uint8_t> *costField, const uint16_t target, Grid<uint16_t> *result);

/**
 * @brief Combines integration fields a and b into c (c can also point to a or b)
 * 
 * @param a An integration field to consider (cannot be NULL)
 * @param b Another integration field to consider (cannot be NULL)
 * @param c The integration field in which to store the result (cannot be NULL)
 */
void CombineIntegrationFields(const Grid<uint16_t> *a, const Grid<uint16_t> *b, Grid<uint16_t> *c);

/**
 * @brief Get the best neighbouring cell that is possible to reach
 * 
 * @param intField integration field to consider (cannot be NULL)
 * @param x x coordinate of cell
 * @param y y coordinate of cell
 * @return uint32_t id of best neigbouring cell
 */
uint16_t GetBestNeighbour(const Grid<uint16_t> *intField, const uint8_t x, const uint8_t y);

/**
 * @brief Gets the best direction to travel in from this cell
 * 
 * @param intfield integration field to consider (cannot be NULL)
 * @param x x coordinate of cell
 * @param y y coordinate of cell
 * @return MovementDirection with best direction of travel
 */
MovementDirection GetBestDirection(const Grid<uint16_t> *intField, const uint8_t x, const uint8_t y);

} // namespace proxima

#endif // PROXIMA_HPP_INCLUDED
