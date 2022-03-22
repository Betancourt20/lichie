
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
tfin = 3
ts = 0.05

# Initial state conditions
q = p560.qn
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
Td = transl(0.5963, -0.1501, 0.6575)@troty(np.pi/2)  # desired Homogen matrix
Td2 = SE3(0.5960, -0.1501, 0.6575)@SE3.Ry(np.pi/2)
vel0 = np.zeros((6,))

# Interaction model variables
v1 = 5
varsgamma = np.array([v1, v1, v1, v1, v1, v1])
VarGamma = np.diag(varsgamma)

nuvs = 12.5
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Dd = np.diag(kvs)

luvs = 100
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp_t = np.diag(kps)

# Motion control variables
nuvs2 = 3.5
kvs2 = np.array([nuvs2, nuvs2, nuvs2, nuvs2, nuvs2, nuvs2])
Kv = np.diag(kvs2)

luvs2 = 5
kps2 = np.array([luvs2, luvs2, luvs2, luvs2, luvs2, luvs2])
Kp = np.diag(kps2)
#########################
T = p560nf.fkine(p560.qn)
T = np.array(T)
pos_T = p560nf.fkine(p560.qn)  # equal to T (Just for the Initial condition)
p = transl(T)
pos0 = np.r_[p, 0, 0, 0]


targs = {'VarGamma': VarGamma, 'Dd': Dd, 'Kp': Kp,
         'pos_T': pos_T, 'pos0': pos0, 'vel0': vel0}


def tr2delta_explicit(T0, T1):
    T0 = np.array(T0)
    T1 = np.array(T1)
    delta = np.array([T1[:3, 3]-T0[:3, 3], 0.5*(np.cross(T0[:3, 0], T1[:3, 0])
                                                + np.cross(T0[:3, 1], T1[:3, 1]) + np.cross(T0[:3, 2], T1[:3, 2]))])
    return delta


def tau(p560nf, t, q, qd, VarGamma, Dd, Kp, pos_T, pos0, vel0):

    M = p560nf.inertia(q)
    g = p560nf.gravload(q)
    C = p560.coriolis(q, qd)
    J = p560nf.jacob0(q)
    T = p560nf.fkine(q)
    if(t >= 0 and t < 2.500):  # First 2 sec
        f = 0
    if(t >= 2.500 and t < 3.500):  # For 4 sec
        f = 0
    if (t >= 3.500):
        f = 0
    Fext = np.array([f, 0, 0, 0, 0, 0])
    Ext_force = p560nf.pay(Fext)

    J_dot = p560nf.jacob_dot(q, qd)
    x_tilde = tr2delta_explicit(Td, pos_T)
    x_tilde = x_tilde.reshape(6,)
    xdot_tilde = vel0
    xdot = J@qd
    # Interaction Model
    acc = np.linalg.pinv(VarGamma)@(-Dd@xdot_tilde
                                    - Kp_t@x_tilde - Fext)
    q_des = p560nf.ikine_LM(pos_T, q)

    #qd_des = np.linalg.pinv(J)@vel0
    # print(qd_des)
    vel = vel0 + acc*t
    pos = pos0 + vel*t

    # print(pos)

    pos_T = SE3(pos[0], pos[1], pos[2]) @ SE3.Ry(np.pi/2)

    nu2 = (Kv@(0-qd) + Kp@(q_des.q - q))
    u = M@nu2 + C@qd + g + Ext_force

    t_list.append(t)
    u_list.append(u)
    p_list.append(transl(np.array(T)))
    return u


t_list = []
u_list = []
p_list = []

print('tau computed')

#  Solving the FD and simulating it
tg = p560nf.fdyn(tfin, q, tau, targs=targs, dt=ts)
print('Computed forward dynamics')

# Plot
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

# Animation of the robt
p560.plot(tg.q)
plt.show()

# print(t_list.shape)
