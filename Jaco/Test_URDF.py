import numpy as np
import roboticstoolbox as rtb
import math
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={
                    'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

# %matplotlib notebook
# %config NotebookBackend.figure_format = 'retina'

# In[1]:
Jaco2 = rtb.models.URDF.KinovaJaco2()

print(Jaco2.fkine(Jaco2.qn))

Jaco2.plot(Jaco2.qn)
plt.show()
