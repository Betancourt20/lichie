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

# In[1]:
p560 = rtb.models.DH.Puma560()
p560nf = p560.nofriction()

# For time simulation
tfin = 5
ts = 0.05

# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
Td = transl(0.5963, -0.1501, 0.6575)@troty(np.pi/2)  # desired Homogen matrix

# ----------------------------------------------------------------
# Control variables
v1 = 5
varsgamma = np.array([v1, v1, v1, v1, v1, v1])
VarGamma = np.diag(varsgamma)

nuvs = 20
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Dd = np.diag(kvs)

luvs = 100
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)

# ----------------------------------------------------------------
# Parameters for the simulation external force
# The function gives 0 external force between t0 and t1
# Then, between t1 and t2 the external force is applied
# This functions has the following format
#  t >= t0 and t < t1,  0
#  t >= t1 and t < t2,  f
#  t>= tu,              0
# This times can be set as the user needs
t0 = 0
t1 = 2.5
t2 = 3.5
f = 20  # Amount of force
# vector of exteranl forces. [fx,fy.fz,fwx, fwy, fwz]
F = np.array([f, 0, 0, 0, 0, 0])

# -------------------------------------------------------------
# Required initial parameters for computing the tau functions
targs = {'VarGamma': VarGamma, 'Dd': Dd, 'Kp': Kp}
t_ant = 0
u = np.zeros((6,))
T = p560nf.fkine(q)


def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def F_ext(t, F, t0, t1, t2):
    if(t >= t0 and t < t1):  # First 2 sec
        Ext_F = np.array([0, 0, 0, 0, 0, 0])
    if(t >= t1 and t < t2):  # For 4 sec
        Ext_F = F
    if (t >= t2):
        Ext_F = np.array([0, 0, 0, 0, 0, 0])
    # Ext_force = p560nf.pay(Fext)
    return Ext_F


def tau(p560nf, t, q, qd, VarGamma, Dd, Kp):
    global t_ant, ts, tr_ant, u, T, F, t0, t1, t2

    if (t >= t_ant + ts):
        M = p560nf.inertia(q)
        g = p560nf.gravload(q)
        C = p560.coriolis(q, qd)
        J = p560nf.jacob0(q)
        T = p560nf.fkine(q)
        Ext_force = F_ext(t, F, t0, t1, t2)
        J_dot = p560nf.jacob_dot(q, qd)
        x_tilde = tr2delta_explicit(Td, T)
        x_tilde = x_tilde.reshape(6,)
        xdot = J@qd
        nu = np.linalg.pinv(J)@np.linalg.pinv(VarGamma)@(-Kp @
                                                         x_tilde - Dd@xdot - VarGamma@J_dot@qd - Ext_force)
        u = M@nu + C@qd + g
        t_ant = t_ant + ts
        u_list.append(u)
        t_list.append(t)
        p_list.append(transl(np.array(T)))

    return u


t_list = []
u_list = []
p_list = []

print('tau computed')

#  Solving the FD and simulating it
sargs = {'max_step': 0.05}
tg = p560nf.fdyn(tfin, q, tau, targs=targs, dt=ts, sargs=sargs)
print('Computed forward dynamics')

# Graph all states
p_list = np.array(p_list)
t_list = np.array(t_list)
u_list = np.array(u_list)

# Joint coordinates graph
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

# Positions coordinates graph
rtb.tools.trajectory.XYZplot(t_list, p_list, labels='x y z')
plt.show()

# Torques graph
rtb.tools.trajectory.tausplot(t_list, u_list)
plt.show()

# Animation of the robt
p560.plot(tg.q)
plt.show()
