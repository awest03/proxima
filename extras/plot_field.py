# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Bonus script to visualise the generated vector field

import sys
import numpy as np
import matplotlib.pyplot as plt

# Load direction data from file
fileName = sys.argv[1]
data = np.genfromtxt(fileName, delimiter=",")

for iy, ix in np.ndindex(data.shape):
    plt.quiver(ix, iy, 0.5*np.cos(data[iy,ix]), -0.5*np.sin(data[iy,ix]))

plt.xticks(range(0, data.shape[1]))
plt.yticks(range(0, data.shape[0]))

plt.gca().invert_yaxis()

plt.show()