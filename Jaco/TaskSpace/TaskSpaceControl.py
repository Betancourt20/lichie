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
jaco = rtb.models.DH.Jaco2()
jaconf = jaco.nofriction()

# For time simulation
tfin = 5
ts = 0.05

# Initial state conditions
q = jaco.qh
qd = np.zeros((6,))
qdd = np.zeros((6,))

# desired Homogen matrix. The rotations are need to be
# in the same frame (base-frame)
# Td = transl(0.4, -0.2184, 0.5414)@trotz(1.624)@troty(1.07)@trotx(-2.91)
Td = SE3(0.4, -0.2184, 0.5414)@SE3.Rz(1.624)@SE3.Ry(1.07)@SE3.Rx(-2.91)
# Control variables
nuvs = 5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 7
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)

# ------------------------------------------------------------------
# Constraints Torques [Nm]
t0max = 40
t1max = 80
t2max = 40
t3max = 20
t4max = 20
t5max = 20
tau_max = np.array([t0max, t1max, t2max, t3max, t4max, t5max])
# Constraints angular velocity [degrees/s]
q0d_max = math.radians(36)
q1d_max = math.radians(36)
q2d_max = math.radians(36)
q3d_max = math.radians(48)
q4d_max = math.radians(48)
q5d_max = math.radians(48)
qd_max = np.array([q0d_max, q1d_max, q2d_max, q3d_max, q4d_max, q5d_max])
# print(qd_max)
# ------------------------------------------------------------------

t_ant = 0
u = jaconf.gravload(q)
targs = {'ts': ts}

# Function to compute the torque


def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def tau(jaconf, t, q, qd, ts):
    global t_ant, u
    if (t > t_ant + ts):
        M = jaconf.inertia(q)
        g = jaconf.gravload(q)
        C = jaco.coriolis(q, qd)
        J = jaconf.jacob0(q)
        T = jaconf.fkine(q)
        J_dot = jaconf.jacob_dot(q, qd)
        x_tilde = tr2delta_explicit(Td, T)
        x_tilde = x_tilde.reshape(6,)
        xdot = J@qd
        nu = np.linalg.pinv(J)@(-Kp@x_tilde - Kv@xdot - J_dot@qd)
        u = M@nu + C@qd + g
        t_list.append(t)
        u_list.append(u)
        p_list.append(transl(np.array(T)))
        t_ant = t_ant + ts
    return u


t_list = []
u_list = []
p_list = []
print('tau computed')
sargs = {'max_step': 0.05}
#  Solving the FD and simulating it
tg = jaconf.fdyn(tfin, q, tau, dt=ts, qd_max=qd_max,
                 tau_max=tau_max, targs=targs, sargs=sargs)
print('Computed forward dynamics')

t_list = np.array(t_list)
u_list = np.array(u_list)
p_list = np.array(p_list)

# Joint coordinates graph
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

# Positions coordinates graph
rtb.tools.trajectory.XYZplot(tg.t, p_list, labels='x y z')
plt.show()

# Torques graph
rtb.tools.trajectory.tausplot(tg.t, u_list)
plt.show()

jaco.plot(tg.q)
plt.show()
