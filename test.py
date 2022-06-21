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
    blockId = p.loadURDF("block.urdf", [0.5, 0, 0.5], p.getQuaternionFromEuler([0, -np.pi/2, 0]),
                         useFixedBase=1, globalScaling=10.0)
    # load the ur5 robot with a robotiq85 hand
    curdir = os.path.dirname(__file__)
    p.setAdditionalSearchPath(os.path.join(curdir, 'urdf'))
    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF("ur5_robotiq_85.urdf", cubeStartPos, cubeStartOrientation)
    print(f"BoxId = {robotId}")
    # init jnt values
    jnt_values = p.calculateInverseKinematics(robotId, endEffectorLinkIndex=4,
                                              targetPosition=[0.5, 0, 0.6],     # local coordinate
                                              currentPositions=list(np.radians([0,-90,90,-90,-90,0,0,0,0,0,0,0])))
    p.resetJointState(robotId, jointIndex=1, targetValue=jnt_values[0])
    p.resetJointState(robotId, jointIndex=2, targetValue=jnt_values[1])
    p.resetJointState(robotId, jointIndex=3, targetValue=jnt_values[2])
    p.resetJointState(robotId, jointIndex=4, targetValue=jnt_values[3])
    p.resetJointState(robotId, jointIndex=5, targetValue=jnt_values[4])
    p.resetJointState(robotId, jointIndex=6, targetValue=jnt_values[5])

    jointnum = p.getNumJoints(robotId)
    print(f"joint_num = {jointnum}")
    jointinfo = p.getJointInfo(robotId, jointIndex=7)
    print(f"joint_info = {jointinfo}")
    # velocity control test
    # maxForce = [500]
    # targetVel = [-np.pi/36]
    # mode = p.VELOCITY_CONTROL
    # p.setJointMotorControlArray(bodyUniqueId=robotId,
    #                             jointIndices=[6],
    #                             controlMode=mode,
    #                             targetVelocities=targetVel,
    #                             forces=maxForce)
    p.enableJointForceTorqueSensor(robotId, jointIndex=6)

    # start the simulation
    for i in range(10000):
        p.stepSimulation()  # default timestep = 1/240 seconds
        jnt_values = p.calculateInverseKinematics(robotId, endEffectorLinkIndex=7,
                                                  targetPosition=[0.5, 0, 1.6-.0001*i],     # world coordinate
                                                  targetOrientation=p.getQuaternionFromEuler([-np.pi/2, 0, np.pi/2]))
        print(f"jnt_values = {jnt_values}")
        mode = p.POSITION_CONTROL
        p.setJointMotorControlArray(robotId,
                                    jointIndices=[1, 2, 3, 4, 5, 6],
                                    controlMode=mode,
                                    targetPositions=jnt_values[:6])
        # print(f"joint6_ft = {p.getJointState(robotId, 6)[2]}")
        time.sleep(1./240.)

    cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
    print(f"pos={cubePos}, orn={cubeOrn}")
    p.disconnect()
