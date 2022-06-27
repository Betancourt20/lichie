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
Jaco = rtb.models.DH.Jaco2()
Jaconf = Jaco.nofriction()

# For time simulation
tfin = 7
ts = 0.05
# mass of the robot = 4.4 kg
# Max linear vel =  20cm/s

# Initial state conditions
q = Jaco.qh
qd = np.zeros((6,))
qdd = np.zeros((6,))
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

# Desired Values
Td = SE3(0.1644, -0.2184, 0.5414)@SE3.Rz(1.624)@SE3.Ry(1.07)@SE3.Rx(-2.91)

# ----------------------------------------------------------------
# Interaction model variables
v1 = 6.36
v2 = 11.14
v3 = 19.10
v4 = 0.21
v5 = 0.21
v6 = 0.21
varsgamma = np.array([v1, v2, v3, v4, v5, v6])
VarGamma = np.diag(varsgamma)

nuvs_1 = 18.18
nuvs_2 = 31.82
nuvs_3 = 54.55
nuvs_4 = 0.85
nuvs_5 = 0.85
nuvs_6 = 0.85
kvs = np.array([nuvs_1, nuvs_2, nuvs_3, nuvs_4, nuvs_5, nuvs_6])
Dd = np.diag(kvs)

luvs = 0
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp_i = np.diag(kps)

# Motion control variables
kvs2 = np.array([10, 10, 10, 10, 10, 10])
Kv = np.diag(kvs2)

kps2 = np.array([0, 0, 0, 0, 0, 0])
#kps2 = np.array([7, 7, 7, 10, 10, 10])
Kp = np.diag(kps2)
# ------------------------------------------------------------------
# ------------------------------------------------------------------

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
f = 0.2  # Amount of force
# vector of exteranl forces. [fx,fy.fz,fwx,fwy,fwz]
F = np.array([-f, 0, 0, 0, 0, 0])  # Here the user can chose the axis
ext_f = np.array([0, 0, 0, 0, 0, 0])  # initial condition
# -------------------------------------------------------------
# Required initial parameters for computing the tau functions
t_ant = 0
t2_ant = 0
u = Jaconf.gravload(q)
T = Jaconf.fkine(q)
pos_T = Jaconf.fkine(q)  # equal to T (Just for the Initial condition)
pos = np.r_[transl(np.array(pos_T)), 0, 0, 0]
vel = np.zeros((6,))
#print(T)
#print(pos_T)
print(q)
targs = {'VarGamma': VarGamma, 'Dd': Dd, 'Kp': Kp,
         'F': F, 't0': t0, 't1': t1, 't2': t2, 'ts': ts}

pargs = {'F': F, 't0': t0, 't1': t1, 't2': t2, 'ts': ts}


def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def F_ext(Jaconf, t, q, F, t0, t1, t2, ts):
    global t2_ant, ext_f
    if (t >= t2_ant + ts):
        J = Jaconf.jacob0(q)  # Jacobian in the inertial frame
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


def tau(Jaconf, t, q, qd, VarGamma, Dd, Kp, F, t0, t1, t2, ts):
    global t_ant, u, T, vel, pos, pos_T
    if (t >= t_ant + ts):
        M = Jaconf.inertia(q)
        g = Jaconf.gravload(q)
        C = Jaco.coriolis(q, qd)
        J = Jaconf.jacob0(q)
        T = Jaconf.fkine(q)
        Ext_force = F_ext(Jaconf, t, q, F, t0, t1, t2, ts)
        x_tilde = tr2delta_explicit(T, pos_T)
        x_tilde = x_tilde.reshape(6,)
        xdot_tilde = vel
      
     # Interaction Model
        acc = np.linalg.pinv(VarGamma)@(-Dd@xdot_tilde
                                        - Kp_i@x_tilde - Ext_force)
                                        
        q_des = Jaconf.ikine_LM(pos_T,q)
        qd_des = np.linalg.pinv(J)@vel                                
        vel = vel + acc*ts
        pos = pos + vel*ts
        pos_T = SE3(pos[0], pos[1], pos[2])@SE3.Rz(1.624)@SE3.Ry(1.07)@SE3.Rx(-2.91) #@ SE3.Ry(np.pi/2)

        nu = (Kv@(qd_des-qd) + Kp@(q_des.q - q))
        u = M@nu + C@qd + g

        t_ant = t_ant + ts
        t_list.append(t)
        u_list.append(u)
        p_list.append(transl(np.array(T)))
        pd_list.append(transl(np.array(pos_T)))
        q_dlist.append(q_des.q)
        qd_list.append(qd)
        qd_dlist.append(qd_des)
          
    return u


t_list = []
u_list = []
p_list = [] #x
pd_list = []# xdot
q_dlist = [] #q
qd_list = [] #q dot
qd_dlist = [] #qd dot

print('tau computed')

#  Solving the FD and simulating it
sargs = {'max_step': 0.05}
tg = Jaconf.fdyn(tfin, q, tau, F_ext, qd_max=qd_max, tau_max=tau_max,
                 targs=targs, pargs=pargs, dt=ts, sargs=sargs)

print('Computed forward dynamics')

t_list = np.array(t_list)
u_list = np.array(u_list)
p_list = np.array(p_list)
pd_list = np.array(pd_list)
q_dlist = np.array(q_dlist)
qd_list = np.array(qd_list)
qd_dlist = np.array(qd_dlist)

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

def Graphs_qdots(t, qd, tt, qd_des):
    fig, ax = plt.subplots()
    ax = plt.subplot(3, 2, 1)
    plt.plot(t, qd[:, 0], label='qd0')
    plt.plot(tt, qd_des[:, 0], label='qd0d')
    plt.grid(True)
    ax.set_ylabel("$qd0 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 2)
    plt.plot(t, qd[:, 1], label='qd1')
    plt.plot(tt, qd_des[:, 1], label='qd1d')
    plt.grid(True)
    ax.set_ylabel("$qd1 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 3)
    plt.plot(t, qd[:, 2], label='qd2')
    plt.plot(tt, qd_des[:, 2], label='qd2d')
    plt.grid(True)
    ax.set_ylabel("$qd2 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 4)
    plt.plot(t, qd[:, 3], label='qd3')
    plt.plot(tt, qd_des[:, 3], label='qd3d')
    plt.grid(True)
    ax.set_ylabel("$qd3 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 5)
    plt.plot(t, qd[:, 4], label='q4')
    plt.plot(tt, qd_des[:, 4], label='q4d')
    plt.grid(True)
    ax.set_ylabel("$qd4 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    ax = plt.subplot(3, 2, 6)
    plt.plot(t, qd[:, 5], label='qd5')
    plt.plot(tt, qd_des[:, 5], label='qd5d')
    plt.grid(True)
    ax.set_ylabel("$qd5 \ [m]$")
    ax.set_xlim(0, max(t))
    ax.set_xlabel("Time (s)")
    ax.legend(loc="upper right")

    plt.show()


Graphs_position(t_list, p_list, pd_list)
Graphs_qdots(tg.t, tg.qd, t_list, qd_dlist)
Graphs_qs(tg.t, tg.q, t_list, q_dlist)

# Torques graph
rtb.tools.trajectory.tausplot(t_list, u_list)
plt.show()

# Animation of the robt
Jaco.plot(tg.q)
plt.show()
