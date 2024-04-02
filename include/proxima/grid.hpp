/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PROXIMA_GRID_INCLUDED
#define PROXIMA_GRID_INCLUDED

#include <cstdint>
#include <vector>
#include <tuple>

namespace proxima
{

template <typename T>
class Grid
{
public:
    Grid(const uint32_t width, const uint32_t height);

    uint32_t width() const;
    uint32_t height() const;
    uint32_t area() const;

    void fill(const T value);

    std::tuple<uint32_t, uint32_t> getCoordinate(const uint32_t index) const;
    T &operator()(const uint32_t x, const uint32_t y);
    const T &operator()(const uint32_t x, const uint32_t y) const;

    uint32_t getIndex(const uint32_t x, const uint32_t y) const;
    T &operator[](const uint32_t index);
    const T &operator[](const uint32_t index) const;

    std::vector<uint32_t> getDirectNeighbours(const uint32_t x, const uint32_t y) const;
    std::vector<uint32_t> getAllNeighbours(const uint32_t x, const uint32_t y) const;

    bool contains(const uint32_t x, const uint32_t y) const;
    bool contains(const uint32_t index) const;
private:
    std::vector<T> m_data;
    const uint32_t m_width, m_height;
};

template <typename T>
inline Grid<T>::Grid(const uint32_t width, const uint32_t height)
    : m_data(width * height), m_width(width), m_height(height)
{
}

template <typename T>
inline uint32_t Grid<T>::width() const
{
    return m_width;
}

template <typename T>
inline uint32_t Grid<T>::height() const
{
    return m_height;
}

template <typename T>
inline uint32_t Grid<T>::area() const
{
    return m_width * m_height;
}

template <typename T>
inline void Grid<T>::fill(const T value)
{
    uint32_t area = m_width * m_height;
    for (uint32_t i = 0; i < area; i++)
    {
        m_data[i] = value;
    }
}

template <typename T>
inline std::tuple<uint32_t, uint32_t> Grid<T>::getCoordinate(const uint32_t index) const
{
    return { index % m_width, index / m_width };
}

template <typename T>
inline T &Grid<T>::operator()(const uint32_t x, const uint32_t y)
{
    return m_data[y * m_width + x];
}

template <typename T>
inline const T &Grid<T>::operator()(const uint32_t x, const uint32_t y) const
{
    return m_data[y * m_width + x];
}

template <typename T>
uint32_t Grid<T>::getIndex(const uint32_t x, const uint32_t y) const
{
    return y * m_width + x;
}

template <typename T>
T &Grid<T>::operator[](const uint32_t index)
{
    return m_data[index];
}

template <typename T>
const T &Grid<T>::operator[](const uint32_t index) const
{
    return m_data[index];
}

template <typename T>
std::vector<uint32_t> Grid<T>::getDirectNeighbours(const uint32_t x, const uint32_t y) const
{
    std::vector<uint32_t> neighbours;
    neighbours.reserve(4);
    if (x > 0)
        neighbours.push_back(getIndex(x - 1, y));
    if (x < m_width - 1)
        neighbours.push_back(getIndex(x + 1, y));
    if (y > 0)
        neighbours.push_back(getIndex(x, y - 1));
    if (y < m_height - 1)
        neighbours.push_back(getIndex(x, y + 1));
    return neighbours;
}

template <typename T>
std::vector<uint32_t> Grid<T>::getAllNeighbours(const uint32_t x, const uint32_t y) const
{
    std::vector<uint32_t> neighbours = getDirectNeighbours(x, y);
    neighbours.reserve(8); // four more neighbours will join us
    
    if (x > 0)
    {
        if (y > 0)
            neighbours.push_back(getIndex(x - 1, y - 1));
        if (y < m_height - 1)
            neighbours.push_back(getIndex(x - 1, y + 1));
    }
    if (x < m_width)
    {
        if (y > 0)
            neighbours.push_back(getIndex(x + 1, y - 1));
        if (y < m_height - 1)
            neighbours.push_back(getIndex(x + 1, y + 1));
    }

    return neighbours;
}

template <typename T>
bool Grid<T>::contains(const uint32_t x, const uint32_t y) const
{
    return x < m_width && y < m_height;
}

template <typename T>
bool Grid<T>::contains(const uint32_t index) const
{
    return index < (m_width * m_height);
}

} // namespace proxima

#endif