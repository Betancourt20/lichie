# Test using the whole Cartesian model of the manipulator
# Not ready yet

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
ts = 0.05
N = math.ceil(tfin/ts)
t = (np.arange(0, N)) * ts
# Initial state conditions
q = p560.qn
qd = np.zeros((6,))

# Desired Values
Td = transl(0.5963, -0.1501, 0.6575)@troty(np.pi/2)  # desired Homogen matrix
vel = np.zeros((6,))

J = p560nf.jacob0(q)
ve = J@qd
T = p560nf.fkine(p560.qn)
T = np.array(T)
p = transl(T)
xe = np.r_[p, 0, 0, 0]


def run_simulation(N, ts, q, qd, ve, xe):

    for k in np.arange(0, N).reshape(-1):
        if(k >= 0 and k < 40):  # First 2 sec
            f = 0
        if(k >= 40 and k < 80):  # For 4 sec
            f = 15
        if (k >= 80):
            f = 0

        Fext = [0, 0, 0, 0, 0, 0]
        Fext = np.array(Fext)

        Lambda = p560.inertia_x(q)
        Mu = p560.coriolis_x(q, qd)
        Fg = p560.gravload_x(q)
        J = p560nf.jacob0(q)

        hc = 0

        ve_dot = np.linalg.pinv(Lambda)@(-Mu@ve - Fg + hc - Fext)
        ve = ve + ts*ve_dot
        xe = xe + ts*ve

        qd = np.linalg.pinv(J)@ve
        q = q + ts*qd

        print(q)
        """
        # Storage Function
        V = 0.5*(x_tilde_d.T@Kp@x_tilde_d) + 0.5 * \
            (xdot_tilde_d.T@Dd@xdot_tilde_d)

        Vdot = xdot_tilde_d.T@Fext - xdot_tilde_d.T@Dd@xdot_tilde_d
        passiv = xdot_tilde_d.T@Fext
        """
        q_list.append(q)
        """
        u_list.append(u)
        p_list.append(transl(T))
        V_list.append(V)
        Vdot_list.append(Vdot)
        passiv_list.append(passiv)
        """
    return q


q_list = []


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


run_simulation(N, ts, q, qd, ve, xe)
Graph_qs(t, q_list)

q_list = np.array(q_list)
p560.plot(q_list)
plt.show()

# print(Lambda)
# print(Mu)
# print(Fg)
