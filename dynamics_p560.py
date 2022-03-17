
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={
                    'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

# %matplotlib notebook
# %config NotebookBackend.figure_format = 'retina'

# In[1]:
p560 = rtb.models.DH.Puma560()
p560nf = p560.nofriction()

# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
q_des = [np.deg2rad(30), 0.7854, 3.1416, np.deg2rad(0), 0.7854, 0]
qd_des = np.zeros((6,))
qdd_des = np.zeros((6,))

# Control variables
nuvs = 1.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 0.9
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)

# Function to compute the torques


def tau(p560nf, t, q, qd):

    M = p560nf.inertia(q)
    g = p560nf.gravload(q)
    C = p560.coriolis(q, qd)

    nu = qdd_des + Kv@(qd_des - qd) + Kp@(q_des - q)
    u = M@nu + C@qd + g
    # print(tau2.shape)  # shape helps to know the size of the variable
    return u


print('tau computed')

#  Solving the FD and simulating it
tg = p560nf.fdyn(5, q, tau, dt=0.05)
print('Computed forward dynamics')

# Plot
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

p560.plot(tg.q)
plt.show()
