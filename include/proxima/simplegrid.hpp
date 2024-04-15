/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PROXIMA_SIMPLEGRID_HPP_INCLUDED
#define PROXIMA_SIMPLEGRID_HPP_INCLUDED

#include <vector>

#include <proxima/grid.hpp>

namespace proxima
{

/**
 * @brief Implementation of Grid<T> using a std::vector as the underlying storage type
 * 
 * @tparam T 
 */
template <typename T>
class SimpleGrid : public Grid<T>
{
public:
    SimpleGrid(const uint8_t width, const uint8_t height);

    T &operator()(const uint8_t x, const uint8_t y) override;
    const T &operator()(const uint8_t x, const uint8_t y) const override;

    T &operator[](const uint16_t index) override;
    const T &operator[](const uint16_t index) const override;

private:
    std::vector<T> m_data;
};

template <typename T>
inline SimpleGrid<T>::SimpleGrid(const uint8_t width, const uint8_t height)
    : Grid<T>(width, height), m_data(width * height)
{
}


template <typename T>
inline T &SimpleGrid<T>::operator()(const uint8_t x, const uint8_t y)
{
    return m_data[Grid<T>::getIndex(x, y)];
}

template <typename T>
inline const T &SimpleGrid<T>::operator()(const uint8_t x, const uint8_t y) const
{
    return m_data[Grid<T>::getIndex(x, y)];
}

template <typename T>
T &SimpleGrid<T>::operator[](const uint16_t index)
{
    return m_data[index];
}

template <typename T>
const T &SimpleGrid<T>::operator[](const uint16_t index) const
{
    return m_data[index];
}

}

#endif