
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
Jaco = rtb.models.DH.Jaco2()
Jaconf = Jaco.nofriction()

print(Jaconf.dynamics())

# print(Jaconf.inertia(Jaconf.qr))
# For time simulation
tfin = 5
h = 0.05
N = math.ceil(tfin/h)
t = (np.arange(0, N)) * h

# Initial state conditions
q = Jaco.qh  # [0, pi/4, pi, 0, pi/4, 0]
qd = np.zeros((6,))
qdd = np.zeros((6,))

print(q)
# Desired Values
# [np.deg2rad(30), 0.7854, 3.1416, np.deg2rad(0), 0.7854, 0]
q_des = [math.radians(50), math.radians(166), math.radians(
    75), math.radians(-118), math.radians(80), 0]
qd_des = np.zeros((6,))
qdd_des = np.zeros((6,))

# Control variables
nuvs = 3.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 7.5
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)

# Function to compute the torques


def tau(p560nf, t, q, qd):

    M = Jaconf.inertia(q)
    g = Jaconf.gravload(q)
    C = Jaco.coriolis(q, qd)

    nu = qdd_des + Kv@(qd_des - qd) + Kp@(q_des - q)
    u = M@nu + C@qd + g
    # print(tau2.shape)  # shape helps to know the size of the variable
    return u


print('tau computed')

#  Solving the FD and simulating it
tg = Jaconf.fdyn(tfin, q, tau, dt=0.05)
print('Computed forward dynamics')

# Plot
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

Jaco.plot(tg.q)
plt.show()
