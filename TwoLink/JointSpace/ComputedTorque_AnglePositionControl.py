
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
twoLink = rtb.models.DH.TwoLink()
twoLinknf = twoLink.nofriction()

print(twoLink)
# Initial state conditions
q = [np.deg2rad(75), -np.deg2rad(30)]
qd = np.zeros((2,))
qdd = np.zeros((2,))

# Desired Values
# [np.deg2rad(00), 0.7854, 3.1416, np.deg2rad(0), 0.7854, 0]
q_des = [np.deg2rad(75), -np.deg2rad(30)]
qd_des = np.zeros((2,))
qdd_des = np.zeros((2,))

print(q)
# Control variables
nuvs = 1.5
kvs = np.array([nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 3
kps = np.array([luvs, luvs])
Kp = np.diag(kps)

# Function to compute the torques


def tau(twoLinknf, t, q, qd):

    M = twoLinknf.inertia(q)
    g = twoLinknf.gravload(q)
    C = twoLink.coriolis(q, qd)

    nu = qdd_des + Kv@(qd_des - qd) + Kp@(q_des - q)
    u = M@nu + C@qd + g
    # print(tau2.shape)  # shape helps to know the size of the variable
    return u


print('tau computed')

#  Solving the FD and simulating it
tg = twoLinknf.fdyn(5, q, tau, dt=0.05)
print('Computed forward dynamics')

# Plot
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

twoLinknf.plot(tg.q)
plt.show()
