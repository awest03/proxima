/*
    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at https://mozilla.org/MPL/2.0/.
*/

#include <iostream>
#include <proxima/proxima.hpp>

int main()
{
    proxima::Grid<uint8_t> grid (10, 10);
    grid.fill(1);

    grid(0, 0) = 255;
    grid(9, 6) = 255;
    grid(8, 6) = 255;
    grid(7, 6) = 255;
    grid(6, 6) = 255;
    grid(5, 6) = 255;

    std::vector<uint32_t> targets = {
        grid.getIndex(9, 9)
    };

    proxima::Grid<float> directionField = proxima::GenerateVectorField(grid, targets);

    for (uint32_t i = 0; i < grid.height(); i++)
    {
        for (uint32_t j = 0; j < grid.width(); j++)
        {
            std::cout << directionField(j, i)/* / float(M_PI)*/;
            if (j != grid.width() - 1) std::cout << ",";
        }
        std::cout << "\n";
    }

    return 0;
}