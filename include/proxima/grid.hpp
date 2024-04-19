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
 * @tparam T Data type stored in grid
 * @tparam C Type used for x and y coordinates (e.g. uint8_t)
 * @tparam I Type used for cell indices, must be able to handle values up to x*y (e.g. uint16_t)
 * @tparam S Signed version of C (e.g. int8_t)
 */
template <typename T, typename C, typename I, typename S>
class Grid
{
public:
    Grid(const C width, const C height);

    const C width, height;
    virtual I area() const;

    virtual void fill(const T value);

    virtual std::tuple<C, C> getCoordinate(const I index) const;
    virtual T &operator()(const C x, const C y) = 0;
    virtual const T &operator()(const C x, const C y) const = 0;

    virtual I getIndex(const C x, const C y) const;
    virtual T &operator[](const I index) = 0;
    virtual const T &operator[](const I index) const = 0;

    virtual std::vector<I> getDirectNeighbours(const C x, const C y) const;
    virtual std::vector<I> getDiagonalNeighbours(const C x, const C y) const;
    virtual std::vector<I> getAllNeighbours(const C x, const C y) const;

    virtual bool contains(const C x, const C y) const;
    virtual bool contains(const I index) const;
};

template <typename T, typename C, typename I, typename S>
inline Grid<T, C, I, S>::Grid(const C width, const C height)
    : width(width), height(height)
{
}

template <typename T, typename C, typename I, typename S>
inline I Grid<T, C, I, S>::area() const
{
    return width * height;
}

template <typename T, typename C, typename I, typename S>
inline void Grid<T, C, I, S>::fill(const T value)
{
    for (I i = 0; i < area(); i++)
    {
        (*this)[i] = value;
    }
}

template <typename T, typename C, typename I, typename S>
inline std::tuple<C, C> Grid<T, C, I, S>::getCoordinate(const I index) const
{
    return { index % width, index / width };
}

template <typename T, typename C, typename I, typename S>
I Grid<T, C, I, S>::getIndex(const C x, const C y) const
{
    return y * width + x;
}

template <typename T, typename C, typename I, typename S>
std::vector<I> Grid<T, C, I, S>::getDirectNeighbours(const C x, const C y) const
{
    std::vector<I> neighbours;
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

template <typename T, typename C, typename I, typename S>
std::vector<I> Grid<T, C, I, S>::getDiagonalNeighbours(const C x, const C y) const
{
    std::vector<I> neighbours;
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

template <typename T, typename C, typename I, typename S>
std::vector<I> Grid<T, C, I, S>::getAllNeighbours(const C x, const C y) const
{
    std::vector<I> neighbours = getDirectNeighbours(x, y);
    std::vector<I> diagonals = getDiagonalNeighbours(x, y);
    neighbours.insert(neighbours.end(), diagonals.begin(), diagonals.end());
    return neighbours;
}

template <typename T, typename C, typename I, typename S>
bool Grid<T, C, I, S>::contains(const C x, const C y) const
{
    return x < width && y < height;
}

template <typename T, typename C, typename I, typename S>
bool Grid<T, C, I, S>::contains(const I index) const
{
    return index < area();
}

} // namespace proxima

#endif