
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
Jaco = rtb.models.DH.Jaco2()
Jaconf = Jaco.nofriction()

# For time simulation
tfin = 5
ts = 0.05

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

# Initial state conditions
q = Jaco.qh  # [0, pi/4, pi, 0, pi/4, 0]
qd = np.zeros((6,))
qdd = np.zeros((6,))

# Desired Values
q_des = [pi/2, 2.9, 1.3, -2.07, 1.4, 0]
# q_des = q
qd_des = np.zeros((6,))
qdd_des = np.zeros((6,))

# Control variables
nuvs = 1.7
kvs = np.array([nuvs, nuvs, nuvs, nuvs, nuvs, nuvs])
Kv = np.diag(kvs)

luvs = 1.3
kps = np.array([luvs, luvs, luvs, luvs, luvs, luvs])
Kp = np.diag(kps)
t_ant = 0
u = Jaconf.gravload(q)
targs = {'ts': ts}


# Function to compute the torques
def tau(Jaconf, t, q, qd, ts):
    global t_ant, u
    if (t > t_ant + ts):
        M = Jaconf.inertia(q)
        g = Jaconf.gravload(q)
        C = Jaco.coriolis(q, qd)

        nu = qdd_des + Kv@(qd_des - qd) + Kp@(q_des - q)
        u = M@nu + C@qd + g
        t_ant = t_ant + ts
    # print(tau2.shape)  # shape helps to know the size of the variable
    return u


print('tau computed')

sargs = {'max_step': 0.05}
#  Solving the FD and simulating it
tg = Jaconf.fdyn(tfin, q, tau, dt=ts, targs=targs,
                 qd_max=qd_max, tau_max=tau_max, sargs=sargs)
print('Computed forward dynamics')

# Plot
rtb.tools.trajectory.qplot(tg.t, tg.q)
plt.show()

Jaco.plot(tg.q)
plt.show()
