import numpy as np
import math
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
from spatialmath.base import *
np.set_printoptions(linewidth=100, formatter={
                    'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

# %matplotlib notebook
# %config NotebookBackend.figure_format = 'retina'

# In[1]:
p560 = rtb.models.DH.Puma560()
p560nf = p560.nofriction()

# For time simulation
tfin = 3
ts = 0.01

# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
# Td = SE3(0.3963, 0.1500, 0.6574)@SE3.Ry(np.pi/2) # This put some errors, I dont know why ?
Td = transl(0.3963, -0.1501, 0.6575)@troty(np.pi/2)  # desired Homogen matrix

# Control variables
nuvs = 3.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 7
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)


# Function to compute the torques
def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def tau(p560nf, t, q, qd):
    M = p560nf.inertia(q)
    g = p560nf.gravload(q)
    C = p560.coriolis(q, qd)
    J = p560nf.jacob0(q)
    T = p560nf.fkine(q)
    J_dot = p560nf.jacob_dot(q, qd)
    x_tilde = tr2delta_explicit(Td, T)
    x_tilde = x_tilde.reshape(6,)
    xdot = J@qd
    nu = np.linalg.pinv(J)@(-Kp@x_tilde - Kv@xdot - J_dot@qd)
    u = M@nu + C@qd + g
    t_list.append(t)
    u_list.append(u)
    p_list.append(transl(np.array(T)))
    return u


t_list = []
u_list = []
p_list = []

print('tau computed')

#  Solving the FD and simulating it
tg = p560nf.fdyn(tfin, q, tau, dt=ts)
print('Computed forward dynamics')


def Graphs_position(t, p_list):
    p_list = np.array(p_list)
    plt.figure()
    pos = plt.plot(t, p_list)
    plt.grid(True)
    plt.xlim(0, max(t))
    plt.xlabel("Time [s]")
    plt.ylabel("$pos \ [m]$")
    plt.legend(pos[:], ["$x$", "$y$", "$z$"])
    plt.show()


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


# Plot
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

Graphs_position(t_list, p_list)
Graph_taus(t_list, u_list)


# p560.plot(tg.q)
# plt.show()
