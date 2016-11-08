#!/usr/bin/env python


# test shapely

from shapely.geometry import box
b = box(0,0,1,1)
print(list(b.exterior.coords))




# test GUI

import numpy as np
from matplotlib import collections as mc
import pylab as pl

lines = [[(0, 1), (1, 1)], [(2, 3), (3, 3)], [(1, 2), (1, 3)]]

print lines
lc = mc.LineCollection(lines, linewidths=2)
fig, ax = pl.subplots()
ax.add_collection(lc)
ax.autoscale()
ax.margins(0.1)
pl.show()

