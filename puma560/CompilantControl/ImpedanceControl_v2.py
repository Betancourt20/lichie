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
tfin = 8
ts = 0.05

# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# ------------------------------------------------------------------
# Constraints Torques [Nm]
t0max = 77
t1max = 133
t2max = 66
t3max = 13
t4max = 12
t5max = 13
tau_max = np.array([t0max, t1max, t2max, t3max, t4max, t5max])
# Constraints angular velocity [degrees/s]
q0d_max = math.radians(82)
q1d_max = math.radians(74)
q2d_max = math.radians(122)
q3d_max = math.radians(228)
q4d_max = math.radians(241)
q5d_max = math.radians(228)
qd_max = np.array([q0d_max, q1d_max, q2d_max, q3d_max, q4d_max, q5d_max])
# print(qd_max)
# ------------------------------------------------------------------

# Desired Values
Td = transl(0.5963, -0.1501, 0.6575)@troty(np.pi/2)  # desired Homogen matrix

# ----------------------------------------------------------------
# Control variables
v1 = 1
varsgamma = np.array([v1, v1, v1, v1, v1, v1])
VarGamma = np.diag(varsgamma)

nuvs = 12.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Dd = np.diag(kvs)

luvs = 80
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
# vector of exteranl forces. [fx,fy.fz,fwx,fwy,fwz]
F = np.array([f, 0, 0, 0, 0, 0])  # Here the user can chose the axis
ext_f = np.array([0, 0, 0, 0, 0, 0])  # initial condition
# -------------------------------------------------------------
# Required initial parameters for computing the tau functions
t_ant = 0
t2_ant = 0
u = p560nf.gravload(q)
T = p560nf.fkine(q)

targs = {'VarGamma': VarGamma, 'Dd': Dd, 'Kp': Kp,
         'F': F, 't0': t0, 't1': t1, 't2': t2, 'ts': ts}

pargs = {'F': F, 't0': t0, 't1': t1, 't2': t2, 'ts': ts}


def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def F_ext(p560nf, t, q, F, t0, t1, t2, ts):
    global t2_ant, ext_f
    if (t >= t2_ant + ts):
        J = p560nf.jacob0(q)  # Jacobian in the inertial frame
        if(t >= t0 and t < t1):  # First 2 sec
            F = np.array([0, 0, 0, 0, 0, 0])
        if(t >= t1 and t < t2):  # For 4 sec
            F = F
        if (t >= t2):
            F = np.array([0, 0, 0, 0, 0, 0])
            t2_ant = t2_ant + ts
        ext_f = F
    # Ext_force = p560nf.pay(Fext)
    return ext_f


def tau(p560nf, t, q, qd, VarGamma, Dd, Kp, F, t0, t1, t2, ts):
    global t_ant, u, T
    if (t >= t_ant + ts):
        M = p560nf.inertia(q)
        g = p560nf.gravload(q)
        C = p560.coriolis(q, qd)
        J = p560nf.jacob0(q)
        T = p560nf.fkine(q)
        Ext_force = F_ext(p560nf, t, q, F, t0, t1, t2, ts)
        J_dot = p560nf.jacob_dot(q, qd)
        x_tilde = tr2delta_explicit(Td, T)
        x_tilde = x_tilde.reshape(6,)
        xdot = J@qd
        nu = np.linalg.pinv(J)@np.linalg.pinv(VarGamma)@(-Kp @
                                                         x_tilde - Dd@xdot - VarGamma@J_dot@qd - Ext_force)
        u = M@nu + C@qd + g

        t_ant = t_ant + ts
        t_list.append(t)
        u_list.append(u)
        p_list.append(transl(np.array(T)))

    return u


t_list = []
u_list = []
p_list = []

print('tau computed')

#  Solving the FD and simulating it
sargs = {'max_step': 0.05}
tg = p560nf.fdyn(tfin, q, tau, F_ext, qd_max=qd_max, tau_max=tau_max,
                 targs=targs, pargs=pargs, dt=ts, sargs=sargs)

print('Computed forward dynamics')

t_list = np.array(t_list)
u_list = np.array(u_list)
p_list = np.array(p_list)

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
