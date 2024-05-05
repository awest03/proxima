/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <iostream>
#include <proxima/proxima.hpp>
#include <proxima/simplegrid.hpp>

int main()
{
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
    }
    return 0;
}