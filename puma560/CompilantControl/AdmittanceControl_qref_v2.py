
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
# For time simulation
tfin = 6
ts = 0.05

# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
# @troty(np.pi/2)  # desired Homogen matrix
Td = transl(0.5960, -0.1501, 0.6575)@troty(np.pi/2)
vel0 = np.zeros((6,))

# -----------------------------------------------------------------
# Interaction model variables
v1 = 0.8
varsgamma = np.array([v1, v1, v1, v1, v1, v1])
VarGamma = np.diag(varsgamma)

nuvs = 15
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Dd = np.diag(kvs)

luvs = 0
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp_t = np.diag(kps)

# Motion control variables
nuvs2 = 10.5
kvs2 = np.array([nuvs2, nuvs2, nuvs2, nuvs2, nuvs2, nuvs2])
Kv = np.diag(kvs2)

luvs2 = 70
kps2 = np.array([luvs2, luvs2, luvs2, luvs2, luvs2, luvs2])
Kp = np.diag(kps2)
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
f = 5  # Amount of force
# vector of exteranl forces. [fx,fy.fz,fwx,fwy,fwz]
F = np.array([f, 0, 0, 0, 0, 0])  # Here the user can chose the axis
ext_f = np.array([0, 0, 0, 0, 0, 0])  # initial condition
# -------------------------------------------------------------
# Required initial parameters for computing the tau functions
t_ant = 0
t2_ant = 0
u = p560nf.gravload(q)
T = p560nf.fkine(q)

targs = {'VarGamma': VarGamma, 'Dd': Dd, 'Kp_t': Kp_t,
         'Kv': Kv, 'Kp': Kp, 'F': F, 't0': t0, 't1': t1, 't2': t2, 'ts': ts, 'Td': Td}

pargs = {'F': F, 't0': t0, 't1': t1, 't2': t2, 'ts': ts}

pos_T = p560nf.fkine(q)  # equal to T (Just for the Initial condition)
pos = np.r_[transl(np.array(pos_T)), 0, 0, 0]
vel = np.zeros((6,))


def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def F_ext(p560nf, t, q, F, t0, t1, t2, ts):
    global t2_ant, ext_f
    if (t >= t2_ant + ts):
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


def tau(p560nf, t, q, qd, VarGamma, Dd, Kp_t, Kv, Kp, F, t0, t1, t2, ts, Td):
    global t_ant, u, T, vel, pos, pos_T

    if (t >= t_ant + ts):
        M = p560nf.inertia(q)
        g = p560nf.gravload(q)
        C = p560.coriolis(q, qd)
        J = p560nf.jacob0(q)
        T = p560nf.fkine(q)
        Ext_force = F_ext(p560nf, t, q, F, t0, t1, t2, ts)
        x_tilde = tr2delta_explicit(T, pos_T)
        x_tilde = x_tilde.reshape(6,)
        #xdot = J@qd

        xdot_tilde = vel
        # xdot = J@qd
        # Interaction Model
        acc = np.linalg.pinv(VarGamma)@(-Dd@xdot_tilde
                                        - Kp_t@x_tilde - Ext_force)
        q_des = p560nf.ikine_LM(pos_T, q)
        qd_des = np.linalg.pinv(J)@vel
        # print(pos, q_des.q)
        vel = vel + acc*ts
        pos = pos + vel*ts
        pos_T = SE3(pos[0], pos[1], pos[2]) @ SE3.Ry(np.pi/2)
        # Main control
        nu = (Kv@(qd_des-qd) + Kp@(q_des.q - q))
        u = M@nu + C@qd + g
        t_ant = t_ant + ts  # update t_ant
        # make the vector outputs
        t_list.append(t)
        u_list.append(u)
        p_list.append(transl(np.array(T)))
        pd_list.append(transl(np.array(pos_T)))
        q_dlist.append(q_des.q)
    return u


t_list = []
u_list = []
p_list = []
pd_list = []
q_dlist = []

print('tau computed')


#  Solving the FD and simulating it
sargs = {'max_step': 0.05}
tg = p560nf.fdyn(tfin, q, tau, F_ext, targs=targs,
                 pargs=pargs, dt=ts, sargs=sargs)

print('Computed forward dynamics')

t_list = np.array(t_list)
u_list = np.array(u_list)
p_list = np.array(p_list)
pd_list = np.array(pd_list)
q_dlist = np.array(q_dlist)

# Joint coordinates graph
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()


def Graphs_position(t, p_list, pd_list):
    fig, ax = plt.subplots()
    ax = plt.subplot(3, 1, 1)
    plt.plot(t, p_list[:, 0], label='x')
    plt.plot(t, pd_list[:, 0], label='xd')
    plt.grid(True)
    ax.set_ylabel("$x \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 1, 2)
    plt.plot(t, p_list[:, 1], label='y')
    plt.plot(t, pd_list[:, 1], label='yd')
    plt.grid(True)
    ax.set_ylabel("$y \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 1, 3)
    plt.plot(t, p_list[:, 2], label='z')
    plt.plot(t, pd_list[:, 2], label='zd')
    plt.grid(True)
    ax.set_ylabel("$z \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    plt.show()


def Graphs_qs(t, q, tt, q_d):
    fig, ax = plt.subplots()
    ax = plt.subplot(3, 2, 1)
    plt.plot(t, q[:, 0], label='q0')
    plt.plot(tt, q_d[:, 0], label='q0d')
    plt.grid(True)
    ax.set_ylabel("$q0 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 2)
    plt.plot(t, q[:, 1], label='q1')
    plt.plot(tt, q_d[:, 1], label='q1d')
    plt.grid(True)
    ax.set_ylabel("$q1 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 3)
    plt.plot(t, q[:, 2], label='q2')
    plt.plot(tt, q_d[:, 2], label='q2d')
    plt.grid(True)
    ax.set_ylabel("$q2 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 4)
    plt.plot(t, q[:, 3], label='q3')
    plt.plot(tt, q_d[:, 3], label='q3d')
    plt.grid(True)
    ax.set_ylabel("$q3 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 5)
    plt.plot(t, q[:, 4], label='q4')
    plt.plot(tt, q_d[:, 4], label='q4d')
    plt.grid(True)
    ax.set_ylabel("$q4 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 6)
    plt.plot(t, q[:, 5], label='q5')
    plt.plot(tt, q_d[:, 5], label='q5d')
    plt.grid(True)
    ax.set_ylabel("$q5 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    plt.show()


# Positions coordinates graph
# rtb.tools.trajectory.XYZplot(t_list, p_list, labels='x y z', stack=True)
# plt.show()
Graphs_position(t_list, p_list, pd_list)

Graphs_qs(tg.t, tg.q, t_list, q_dlist)

# Torques graph
rtb.tools.trajectory.tausplot(t_list, u_list)
plt.show()

# Animation of the robt
p560.plot(tg.q)
plt.show()

# print(t_list.shape)

# print(t_list.shape)
