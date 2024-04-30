/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <proxima/mesh.hpp>
#include <polypartition/polypartition.h>

using namespace proxima;

bool Vertex::operator==(const Vertex &other) const
{
    return x == other.x && y == other.y;
}


Polygon::Polygon(const std::vector<Vertex> &borderVertices)
    : m_vertices(borderVertices)
{
    m_offsets.push_back(0);
}

void Polygon::insertIsland(const std::vector<Vertex> &islandVertices)
{
    if (!m_isWritingIsland)
    {
        uint32_t offset = m_vertices.size();
        m_vertices.insert(m_vertices.end(), islandVertices.begin(), islandVertices.end());
        m_offsets.push_back(offset);
    }
    else
        throw "You're writing another island at the same time!";
}

void Polygon::startIsland(uint16_t reservation)
{
    m_vertices.reserve(reservation);
    m_offsets.push_back(m_vertices.size());
    m_isWritingIsland = true;
}

void Polygon::pushIslandVertex(Vertex vert)
{
    if (m_isWritingIsland)
        m_vertices.push_back(vert);
    else
        throw "You forgot to call Polygon::startIsland!";
}

void Polygon::finishIsland()
{
    m_isWritingIsland = false;
}

PolygonPart Polygon::operator[](uint32_t poly)
{
    PolygonPart pp;
    auto offset = m_offsets[poly];
    pp.vertices = &m_vertices[offset];
    pp.end = (poly == m_offsets.size() - 1) ? m_vertices.size() - offset : m_offsets[poly+1];
    return pp;
}

uint32_t Polygon::polyCount() const
{
    return m_offsets.size();
}


std::optional<uint16_t> Mesh::findVertex(Vertex vert)
{
    for (uint16_t i = 0; i < m_vertices.size(); i++)
    {
        if (m_vertices[i] == vert) return i;
    }
    return {};
}

uint16_t Mesh::addVertex(Vertex vert)
{
    m_vertices.push_back(vert);
    return m_vertices.size() - 1;
}

uint16_t Mesh::addTriangle(uint16_t a, uint16_t b, uint16_t c)
{
    m_indices.push_back(a);
    m_indices.push_back(b);
    m_indices.push_back(c);
    return m_indices.size() - 3;
}

Vertex &Mesh::operator[](uint16_t index)
{
    return m_vertices[index];
}

const Vertex &Mesh::operator[](uint16_t index) const
{
    return m_vertices[index];
}

uint16_t &Mesh::operator()(uint16_t triangle, uint16_t point)
{
    return m_indices[triangle * 3 + point];
}

const uint16_t &Mesh::operator()(uint16_t triangle, uint16_t point) const
{
    return m_indices[triangle * 3 + point];
}

uint16_t Mesh::vertexCount() const
{
    return m_vertices.size();
}

uint16_t Mesh::indexCount() const
{
    return m_indices.size();
}

uint16_t Mesh::triangleCount() const
{
    return m_indices.size() / 3;
}

Mesh proxima::GenerateMeshFromPolygon(Polygon &poly)
{
    // Convert from proxima polygon to format used by polypartition
    std::list<TPPLPoly> polys, result;
    for (uint32_t i = 0; i < poly.polyCount(); i++)
    {
        PolygonPart pp = poly[i];
        TPPLPoly p;
        p.Init(pp.end);
        for (uint32_t j = 0; j < pp.end; j++)
        {
            p[j].x = pp.vertices[j].x;
            p[j].y = pp.vertices[j].y;
        }
        if (i > 0) p.SetHole(true);
        else p.SetHole(false);
        p.SetOrientation(TPPL_ORIENTATION_CCW);
        polys.push_back(p);
    }
    // Fill polygon with triangles
    TPPLPartition tpp;
    if (tpp.Triangulate_EC(&polys, &result) == 0)
    {
        return Mesh();
    }
    // Generate mesh from result
    Mesh mesh;
    for (auto &p : result)
    {
        uint16_t a, b, c;
        for (long i = 0; i < p.GetNumPoints(); i++)
        {
            proxima::Vertex v = { (float)p[i].x, (float)p[i].y };
            auto idopt = mesh.findVertex(v);
            uint16_t id;
            if (idopt.has_value())
                id = idopt.value();
            else
                id = mesh.addVertex(v);
            a = b;
            b = c;
            c = id;
        }
        mesh.addTriangle(a, b, c);
    }
    return mesh;
}