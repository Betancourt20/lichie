
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
Jaco = rtb.models.DH.Jaco()
Jaconf = Jaco.nofriction()

print(Jaconf.dynamics())
print(Jaconf.inertia(Jaconf.qr))
# For time simulation
tfin = 5
h = 0.05
N = math.ceil(tfin/h)
t = (np.arange(0, N)) * h

# Initial state conditions
q = Jaco.qr  # [0, pi/4, pi, 0, pi/4, 0]
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
# [np.deg2rad(30), 0.7854, 3.1416, np.deg2rad(0), 0.7854, 0]
q_des = [0, pi/4, pi, 0, pi/4, 0]
qd_des = np.zeros((6,))
qdd_des = np.zeros((6,))

# Control variables
nuvs = 1.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 3
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)

# Function to compute the torques


def run_simulation(N, ts, q, qd, q_des):

    for k in np.arange(0, N).reshape(-1):

        M = Jaconf.inertia(q)
        g = Jaconf.gravload(q)
        C = Jaco.coriolis(q, qd)

        nu = Kv@(qd_des-qd) + Kp@(q_des - q)
        u = M@nu + C@qd + g

        qdd = np.linalg.pinv(M)@(u - C@qd - g)
        qd = qd + qdd*ts
        q = q + qd*ts
        q_list.append(q)
        u_list.append(u)

    return q


q_list = []
u_list = []
print('simulation done')


def Graph_taus(t, u_list):
    u_list = np.array(u_list)
    plt.figure()
    torques = plt.plot(t, u_list)
    plt.grid(True)
    plt.xlim(0, max(t))
    plt.xlabel("Time [s]")
    plt.ylabel("$torques \ [m]$")
    plt.legend(torques[:], [r"$\tau_{1}$", r"$\tau_{2}$", r"$\tau_{3}$",
               r"$\tau_{4}$", r"$\tau_{5}$", r"$\tau_{6}$"])
    plt.show()


def Graph_qs(t, q_list):
    q_list = np.array(q_list)
    plt.figure()
    qs = plt.plot(t, q_list)
    plt.grid(True)
    plt.xlim(0, max(t))
    plt.xlabel("Time [s]")
    plt.ylabel("$q \ [rad]$")
    plt.legend(qs[:], [r"$q_{1}$", r"$q_{2}$", r"$q_{3}$",
                       r"$q_{4}$", r"$q_{5}$", r"$q_{6}$"])
    plt.show()


run_simulation(N, h, q, qd, q_des)
Graph_qs(t, q_list)
Graph_taus(t, u_list)

# Animation of the robt
q_list = np.array(q_list)
Jaco.plot(q_list)
plt.show()
