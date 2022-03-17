#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm

np.set_printoptions(linewidth=100, formatter={
                    'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

puma = rtb.models.DH.Puma560()

print(puma)


# Parameters method
tfin = 3
h = 1e-3
N = np.ceil((tfin-h)/h)
t = h + (np.arange(0, N+1)) * h
k = 0

tau = puma.rne(puma.qn, np.zeros((6,)), np.zeros((6,)))
M = puma.inertia(puma.qn)
C = puma.coriolis(puma.qn, 0.1 * np.ones((6,)))
g = puma.gravload(puma.qn)

qdd = puma.accel(puma.qn, np.zeros((6,)), np.zeros((6,)))
# print(np.zeros((6,)))
# Then here I need to derivate two times qdd to obtain the velocity and position


# Initial conditions
q_dot = [0, 0, 0, 0, 0, 0]
q = [0, 0, 0, 0, 0, 0]
# print(q_dot)
# for k in np.arange(1, N+1).reshape(-1):
#    print(k)
#q_dot = q_dot + h*qdd
#q = q + h*q_dot
#puma.plot(q, block=False)
# plt.show()
