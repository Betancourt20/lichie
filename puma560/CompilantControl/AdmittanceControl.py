
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
tfin = 5
h = 0.05
N = math.ceil(tfin/h)
t = (np.arange(0, N)) * h
# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
Td = transl(0.5963, -0.1501, 0.6575)@troty(np.pi/2)  # desired Homogen matrix
vel = np.zeros((6,))

# Interaction model variables
v1 = 0.8
varsgamma = np.array([v1, v1, v1, v1, v1, v1])
VarGamma = np.diag(varsgamma)

nuvs = 12.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Dd = np.diag(kvs)

luvs = 100
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)

# Motion control variables
v2 = 0.8
varsgamma2 = np.array([v2, v2, v2, v2, v2, v2])
Md = np.diag(varsgamma2)

nuvs2 = 15
kvs2 = np.array([nuvs2, nuvs2, nuvs2, nuvs2, nuvs2, nuvs2])
Dd_t = np.diag(kvs2)

luvs2 = 100
kps2 = np.array([luvs2, luvs2, luvs2, luvs2, luvs2, luvs2])
Kp_t = np.diag(kps2)

T = p560nf.fkine(q)
T = np.array(T)
pos_T = T  # equal to T (Just for the Initial condition)
pos_T = np.array(pos_T)
p = transl(T)
pos = np.r_[p, 0, 0, 0]


def tr2delta_explicit(T0, T1):
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def run_simulation(N, ts, q, qd, pos_T, vel, pos):
    for k in np.arange(0, N).reshape(-1):
        if(k >= 0 and k < 40):  # First 2 sec
            f = 0
        if(k >= 40 and k < 80):  # For 4 sec
            f = 15
        if (k >= 80):
            f = 0
        Fext = [0, f, 0, 0, 0, 0]
        Fext = np.array(Fext)

        M = p560nf.inertia(q)
        g = p560nf.gravload(q)
        C = p560.coriolis(q, qd)
        J = p560nf.jacob0(q)
        T = p560nf.fkine(q)
        T = np.array(T)
        J_dot = p560nf.jacob_dot(q, qd)

        x_tilde_d = tr2delta_explicit(pos_T, Td)
        x_tilde_d = x_tilde_d.reshape(6,)
        xdot_tilde_d = -vel
        xdot = J@qd
        p = transl(T)

        # Interaction Model
        acc = np.linalg.pinv(VarGamma)@(Dd@xdot_tilde_d + Kp@x_tilde_d - Fext)
        #acc = np.linalg.pinv(VarGamma)@(Dd@xdot_tilde_d - Fext)

        vel = vel + acc*ts
        pos = pos + vel*ts
        pos_T = delta2tr(pos)@troty(np.pi/2)

        # Motion Control
        xdot_tilde_t = vel - xdot
        x_tilde_t = tr2delta_explicit(T, pos_T)
        x_tilde_t = x_tilde_t.reshape(6,)
        nu2 = np.linalg.pinv(J)@np.linalg.pinv(Md)@(Dd_t@xdot_tilde_t
                                                    + Kp_t@x_tilde_t - Md@J_dot@qd - Fext)

        u = M@nu2 + C@qd + g - J.T@Fext

        qdd = np.linalg.pinv(M)@(u - C@qd - g + J.T@Fext)
        qd = qd + qdd*ts
        q = q + qd*ts

        q_list.append(q)
        u_list.append(u)
        p_list.append(p)

    return q


q_list = []
qd_list = []
u_list = []
p_list = []
print('simulation done')


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


run_simulation(N, h, q, qd, pos_T, vel, pos)
Graph_qs(t, q_list)
Graphs_position(t, p_list)
Graph_taus(t, u_list)

# Animation of the robt
q_list = np.array(q_list)
p560.plot(q_list)
plt.show()
plt.show()
plt.show()
plt.show()
plt.show()
plt.show()
