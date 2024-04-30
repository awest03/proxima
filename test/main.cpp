/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <iostream>
#include <proxima/proxima.hpp>
#include <proxima/simplegrid.hpp>
#include <proxima/mesh.hpp>
#include <polypartition/polypartition.h>
#include <list>

int main()
{/*
    proxima::SimpleGrid<uint8_t> grid (10, 10);
    grid.fill(1);

    grid(0, 0) = 255;
    grid(9, 6) = 255;
    grid(8, 6) = 255;
    grid(7, 6) = 255;
    grid(6, 6) = 255;
    grid(5, 6) = 255;

    proxima::SimpleGrid<uint16_t> int1 (10, 10);
    proxima::GenerateIntegrationField(&grid, grid.getIndex(6, 9), &int1);

    proxima::SimpleGrid<uint16_t> int2 (10, 10);
    proxima::GenerateIntegrationField(&grid, grid.getIndex(4, 5), &int2);

    proxima::CombineIntegrationFields(&int1, &int2, &int1);

    proxima::SimpleGrid<proxima::MovementDirection> directionField (10, 10);
    for (uint32_t i = 0; i < directionField.area(); i++)
    {
        auto [x, y] = directionField.getCoordinate(i);
        directionField[i] = proxima::GetBestDirection(&int1, x, y);
    }

    for (uint8_t i = 0; i < grid.height; i++)
    {
        for (uint8_t j = 0; j < grid.width; j++)
        {
            std::cout << directionField(j, i).getAngle();
            if (j != grid.width - 1) std::cout << ",";
        }
        std::cout << "\n";
    }*/

    std::vector<proxima::Vertex> border (4);
    border[0].x = 0.0f;
    border[0].y = 0.0f;
    border[1].x = 1.0f;
    border[1].y = 0.0f;
    border[2].x = 1.0f;
    border[2].y = -1.0f;
    border[3].x = 0.0f;
    border[3].y = -1.0f;
    proxima::Polygon poly (border);
    poly.startIsland(3);
    poly.pushIslandVertex({ 0.25, -0.25 });
    poly.pushIslandVertex({ 0.75, -0.75 });
    poly.pushIslandVertex({ 0.25, -0.75 });
    poly.finishIsland();
    proxima::Mesh mesh = proxima::GenerateMeshFromPolygon(poly);

    for (uint16_t i = 0; i < mesh.vertexCount(); i++)
    {
        std::cout << "[ " << i << " ] ( " << mesh[i].x << ", " << mesh[i].y << " )\n";
    }
    for (uint16_t i = 0; i < mesh.triangleCount(); i++)
    {
        std::cout << "{ " << mesh(i, 0) << ", " << mesh(i, 1) << ", " << mesh(i, 2) << " }\n";
    }


    return 0;
}