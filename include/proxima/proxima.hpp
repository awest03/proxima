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
 * @brief Generates the integration field due to target (the cell index)
 * 
 * @param costField The cost field to consider (cannot be NULL)
 * @param target The target/goal of the pathfinding
 * @param result Grid in which to store the result (cannot be NULL). Must have same width and height as the cost field
 */
void GenerateIntegrationField(const Grid<uint8_t> *costField, const uint32_t target, Grid<uint16_t> *result);

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
uint32_t GetBestNeighbour(const Grid<uint16_t> *intField, const uint32_t x, const uint32_t y);

/**
 * @brief Generates the vector direction field due to the specified integration field
 * 
 * @param integrationField Integration field to consider (cannot be NULL)
 * @param result Grid to store the result (cannot be NULL). Must have same width and height as the integration field
 */
void GenerateVectorField(const Grid<uint16_t> *integrationField, Grid<float> *result);

} // namespace proxima

#endif // PROXIMA_HPP_INCLUDED
