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

/**
 * @brief Simple interface for a generic grid. Default implementation assume row-major list for storage
 * 
 * @tparam T 
 */
template <typename T>
class Grid
{
public:
    Grid(const uint8_t width, const uint8_t height);

    const uint8_t width, height;
    virtual uint16_t area() const;

    virtual void fill(const T value);

    virtual std::tuple<uint8_t, uint8_t> getCoordinate(const uint16_t index) const;
    virtual T &operator()(const uint8_t x, const uint8_t y) = 0;
    virtual const T &operator()(const uint8_t x, const uint8_t y) const = 0;

    virtual uint16_t getIndex(const uint8_t x, const uint8_t y) const;
    virtual T &operator[](const uint16_t index) = 0;
    virtual const T &operator[](const uint16_t index) const = 0;

    virtual std::vector<uint16_t> getDirectNeighbours(const uint8_t x, const uint8_t y) const;
    virtual std::vector<uint16_t> getDiagonalNeighbours(const uint8_t x, const uint8_t y) const;
    virtual std::vector<uint16_t> getAllNeighbours(const uint8_t x, const uint8_t y) const;

    virtual bool contains(const uint8_t x, const uint8_t y) const;
    virtual bool contains(const uint16_t index) const;
};

template <typename T>
inline Grid<T>::Grid(const uint8_t width, const uint8_t height)
    : width(width), height(height)
{
}

template <typename T>
inline uint16_t Grid<T>::area() const
{
    return width * height;
}

template <typename T>
inline void Grid<T>::fill(const T value)
{
    for (uint16_t i = 0; i < area(); i++)
    {
        (*this)[i] = value;
    }
}

template <typename T>
inline std::tuple<uint8_t, uint8_t> Grid<T>::getCoordinate(const uint16_t index) const
{
    return { index % width, index / width };
}

template <typename T>
uint16_t Grid<T>::getIndex(const uint8_t x, const uint8_t y) const
{
    return y * width + x;
}

template <typename T>
std::vector<uint16_t> Grid<T>::getDirectNeighbours(const uint8_t x, const uint8_t y) const
{
    std::vector<uint16_t> neighbours;
    neighbours.reserve(4);
    if (x > 0)
        neighbours.push_back(getIndex(x - 1, y));
    if (x < width - 1)
        neighbours.push_back(getIndex(x + 1, y));
    if (y > 0)
        neighbours.push_back(getIndex(x, y - 1));
    if (y < height - 1)
        neighbours.push_back(getIndex(x, y + 1));
    return neighbours;
}

template <typename T>
std::vector<uint16_t> Grid<T>::getDiagonalNeighbours(const uint8_t x, const uint8_t y) const
{
    std::vector<uint16_t> neighbours;
    neighbours.reserve(4);
    
    if (x > 0)
    {
        if (y > 0)
            neighbours.push_back(getIndex(x - 1, y - 1));
        if (y < height - 1)
            neighbours.push_back(getIndex(x - 1, y + 1));
    }
    if (x < width)
    {
        if (y > 0)
            neighbours.push_back(getIndex(x + 1, y - 1));
        if (y < height - 1)
            neighbours.push_back(getIndex(x + 1, y + 1));
    }

    return neighbours;
}

template <typename T>
std::vector<uint16_t> Grid<T>::getAllNeighbours(const uint8_t x, const uint8_t y) const
{
    std::vector<uint16_t> neighbours = getDirectNeighbours(x, y);
    std::vector<uint16_t> diagonals = getDiagonalNeighbours(x, y);
    neighbours.insert(neighbours.end(), diagonals.begin(), diagonals.end());
    return neighbours;
}

template <typename T>
bool Grid<T>::contains(const uint8_t x, const uint8_t y) const
{
    return x < width && y < height;
}

template <typename T>
bool Grid<T>::contains(const uint16_t index) const
{
    return index < area();
}

} // namespace proxima

#endif