import roboticstoolbox as rtb
from spatialmath import SE3
from PIL import Image
# instantiate 3D browser-based visualizer
from roboticstoolbox.backends.swift import Swift

# modelo con DH
#robot = rtb.models.DH.Panda()
# print(robot)

# Modelo con urdf
robot = rtb.models.URDF.Panda()  # load URDF version of the Panda
print(robot)    # display the model

T = robot.fkine(robot.qz)  # forward kinematics
print(T)

T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
sol = robot.ikine_LM(T)         # solve IK
print(sol)


q_pickup = sol.q
# FK shows that desired end-effector pose was achieved
print(robot.fkine(q_pickup))

qt = rtb.jtraj(robot.qz, q_pickup, 50)

backend = Swift()
backend.launch()            # activate it
backend.add(robot)          # add robot to the 3D scene
for qk in qt.q:             # for each joint configuration on trajectory
    robot.q = qk          # update the robot state
    backend.step()        # update visualization

#qt = rtb.jtraj(robot.qz, q_pickup, 50)
#robot.plot(qt.q, movie='panda1.gif')
