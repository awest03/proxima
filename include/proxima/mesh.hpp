/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#ifndef PROXIMA_MESH_INCLUDED
#define PROXIMA_MESH_INCLUDED

#include <cstdint>
#include <vector>
#include <optional>

namespace proxima
{

struct Vertex
{
    float x = 0.0f, y = 0.0f;

    bool operator==(const Vertex &other) const;
};

struct PolygonPart
{
    Vertex *vertices;
    uint16_t end;

    enum class Orientation : int
    {
        CW = -1,
        None = 0,
        CCW = 1
    };

    Orientation getOrientation() const;
    void setOrientation(Orientation orientation);
    void invert();
};

class Polygon
{
public:
    Polygon(const std::vector<Vertex> &borderVertices);

    void insertIsland(const std::vector<Vertex> &islandVertices);
    void startIsland(uint16_t reservation);
    void pushIslandVertex(Vertex vert);
    void finishIsland();

    // the 0th polygon is the border
    PolygonPart operator[](uint32_t poly);
    uint32_t polyCount() const;

private:
    bool m_isWritingIsland = false;
    std::vector<Vertex> m_vertices;
    std::vector<uint32_t> m_offsets;
};

class Mesh
{
public:
    std::optional<uint16_t> findVertex(Vertex vert);
    uint16_t addVertex(Vertex vert);
    uint16_t addTriangle(uint16_t a, uint16_t b, uint16_t c);

    Vertex &operator[](uint16_t index);
    const Vertex &operator[](uint16_t index) const;
    uint16_t &operator()(uint16_t triangle, uint16_t point);
    const uint16_t &operator()(uint16_t triangle, uint16_t point) const;

    uint16_t vertexCount() const;
    uint16_t indexCount() const;
    uint16_t triangleCount() const;

private:
    std::vector<Vertex> m_vertices;
    std::vector<uint16_t> m_indices;
};

Mesh GenerateMeshFromPolygon(Polygon &poly);

}

#endif