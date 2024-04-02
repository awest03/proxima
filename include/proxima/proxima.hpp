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

Grid<uint16_t> GenerateIntegrationField(const Grid<uint8_t> &costField, const std::vector<uint32_t> &targets);

Grid<float> GenerateVectorField(const Grid<uint16_t> &integrationField);

Grid<float> GenerateVectorField(const Grid<uint8_t> &costField, const std::vector<uint32_t> &targets);

} // namespace proxima

#endif // PROXIMA_HPP_INCLUDED
