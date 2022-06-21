import pybullet as p
import pybullet_data
import time
import os
import numpy as np

if __name__ == '__main__':

    physicsClient = p.connect(p.GUI)                            # or p.DIRECT for non-graphical version
    # load the plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())      # optionally
    print(f"PyBulletSearchPath = {pybullet_data.getDataPath()}")
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    print(f"PlaneId = {planeId}")
    groundbaseId = p.loadURDF("block.urdf", [0, 0, 0.5], p.getQuaternionFromEuler([0, -np.pi / 2, 0]),
                              useFixedBase=1, globalScaling=10.0)
    blockId = p.loadURDF("block.urdf", [0.5, 0, 0.1], p.getQuaternionFromEuler([0, -np.pi/2, 0]),
                         useFixedBase=1, globalScaling=20.0)
    # load the ur5 robot with a robotiq85 hand
    curdir = os.path.dirname(__file__)
    p.setAdditionalSearchPath(os.path.join(curdir, 'urdf'))
    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("ur5.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=1)
    print(f"BoxId = {robotId}")
    # init jnt values
    jnt_values = p.calculateInverseKinematics(robotId, endEffectorLinkIndex=6,
                                              targetPosition=[0.5, 0, 0.2],     # local coordinate
                                              currentPositions=list(np.radians([0,-90,90,-90,-90,0])))
    p.resetJointState(robotId, jointIndex=0, targetValue=jnt_values[0])
    p.resetJointState(robotId, jointIndex=1, targetValue=jnt_values[1])
    p.resetJointState(robotId, jointIndex=2, targetValue=jnt_values[2])
    p.resetJointState(robotId, jointIndex=3, targetValue=jnt_values[3])
    p.resetJointState(robotId, jointIndex=4, targetValue=jnt_values[4])
    p.resetJointState(robotId, jointIndex=5, targetValue=jnt_values[5])

    jointnum = p.getNumJoints(robotId)
    print(f"joint_num = {jointnum}")
    jointinfo = p.getJointInfo(robotId, jointIndex=6)
    print(f"joint_info = {jointinfo}")

    # enable the eef ft_sensor
    p.enableJointForceTorqueSensor(robotId, jointIndex=6)

    # start the simulation
    for i in range(1000):
        p.stepSimulation()  # default timestep = 1/240 seconds
        jnt_values = p.calculateInverseKinematics(robotId, endEffectorLinkIndex=6,
                                                  targetPosition=[0.5, 0, 1.2 - .0001 * i],  # world coordinate
                                                  targetOrientation=p.getQuaternionFromEuler([0, np.pi/2, 0]))
        mode = p.POSITION_CONTROL
        p.setJointMotorControlArray(robotId,
                                    jointIndices=[0, 1, 2, 3, 4, 5],
                                    controlMode=mode,
                                    targetPositions=jnt_values[:6])
        jnt_react_forces = p.getJointState(robotId, 6)[2]

        # if i % 100 == 0:
        #     print(f"eef_ft = {jnt_react_forces}")
        time.sleep(1. / 240.)

    target_Fx = 0.0
    sumError_Fx = 0.0
    lastError_Fx = 0.0
    target_Fy = 0.0
    sumError_Fy = 0.0
    lastError_Fy = 0.0
    target_Fz = 5.0
    sumError_Fz = 0.0
    lastError_Fz = 0.0
    target_Mx = 0.0
    sumError_Mx = 0.0
    lastError_Mx = 0.0
    target_My = 0.0
    sumError_My = 0.0
    lastError_My = 0.0
    target_Mz = 0.0
    sumError_Mz = 0.0
    lastError_Mz = 0.0
    kp, ki, kd = 5e-6, 0, 5e-6
    kp2, ki2, kd2 = 5e-6, 0, 5e-6
    targetPosition = np.array([0.5, 0, 1.05])
    targetOrientation = np.array([0, np.pi/2, 0])    # p.getQuaternionFromEuler([0, np.pi / 2, 0])

    for i in range(10000):
        p.stepSimulation()  # default timestep = 1/240 seconds

        jnt_values = p.calculateInverseKinematics(robotId, endEffectorLinkIndex=6,
                                                  targetPosition=list(targetPosition),  # world coordinate
                                                  targetOrientation=p.getQuaternionFromEuler(targetOrientation))
        # print(f"jnt_values = {jnt_values}")
        mode = p.POSITION_CONTROL
        p.setJointMotorControlArray(robotId,
                                    jointIndices=[0, 1, 2, 3, 4, 5],
                                    controlMode=mode,
                                    targetPositions=jnt_values[:6])

        jnt_react_forces = p.getJointState(robotId, 6)[2]
        if i % 10 == 0:
            print(f"目标位置 = {targetPosition}")
            print(f"目标姿态 = {targetOrientation}")
            print(f"eef_ft = {jnt_react_forces}")

        error_Fx = -jnt_react_forces[2] - target_Fx
        sumError_Fx += error_Fx
        dx = kp * error_Fx + ki * sumError_Fx + kd * (error_Fx - lastError_Fx)
        # print(f"x调节量 = {dx}")
        lastError_Fx = error_Fx
        targetPosition += np.array([dx, 0, 0])

        error_Fy = jnt_react_forces[1] - target_Fy
        sumError_Fy += error_Fy
        dy = kp * error_Fy + ki * sumError_Fy + kd * (error_Fy - lastError_Fy)
        # print(f"y调节量 = {dy}")
        lastError_Fy = error_Fy
        targetPosition += np.array([0, dy, 0])

        error_Fz = jnt_react_forces[0] - target_Fz
        sumError_Fz += error_Fz
        dz = kp * error_Fz + ki * sumError_Fz + kd * (error_Fz-lastError_Fz)
        # print(f"z调节量 = {dz}")
        lastError_Fz = error_Fz
        targetPosition += np.array([0, 0, dz])

        error_Mx = -jnt_react_forces[5] - target_Mx
        sumError_Mx += error_Mx
        da = kp2 * error_Mx + ki2 * sumError_Mx + kd2 * (error_Mx - lastError_Mx)
        # print(f"roll调节量 = {da}")
        lastError_Mx = error_Mx
        targetOrientation += np.array([da, 0, 0])

        error_My = jnt_react_forces[4] - target_My
        sumError_My += error_My
        db = kp2 * error_My + ki2 * sumError_My + kd2 * (error_My - lastError_My)
        # print(f"pitch调节量 = {db}")
        lastError_My = error_My
        targetOrientation += np.array([0, db, 0])

        error_Mz = jnt_react_forces[3] - target_Mz
        sumError_Mz += error_Mz
        dc = kp2 * error_Mz + ki2 * sumError_Mz + kd2 * (error_Mz - lastError_Mz)
        # print(f"yaw调节量 = {dc}")
        lastError_Mz = error_Mz
        targetOrientation += np.array([0, 0, dc])

        time.sleep(1./240.)

    cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
    print(f"pos={cubePos}, orn={cubeOrn}")
    p.disconnect()
