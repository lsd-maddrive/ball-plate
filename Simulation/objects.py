import pybullet as p
import math as m
import numpy as np
import random as rand
import os

class BallOnPlateEnv:
    def __init__(self):
        # Parameters

        # gym placeholdert
        self.observation_space_shape    = [8]
        self.action_space_shape         = [2]
        self.action_space_low           = -1
        self.action_space_high          = 1

class BallOnPlate:

    def __init__(self, showGUI=False, randomInitial=False):
        self.dt = 1/100.
        self.controlAngleLimit = 20
        self.plateSize = 0.2
        self.randomInitial = randomInitial

        self.intial_pos     = np.array([0., 0.])
        self.ballPosition   = self.intial_pos * self.plateSize
        self.ballHeight     = .28

        self.env            = BallOnPlateEnv()

        # Now work with simulator
        if showGUI:
            self.physId = p.connect(p.GUI)
        else:
            self.physId = p.connect(p.DIRECT)

        p.resetSimulation(physicsClientId=self.physId)

        p.setGravity(0,0,-9.81, physicsClientId=self.physId)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, physicsClientId=self.physId)

        ballRadius = 52.2 / 2 / 1000
        ballMass = 0.205
        ballInertia = 2./5*ballMass*ballRadius*ballRadius

        self.plateId = p.loadURDF(os.path.dirname(os.path.realpath(__file__)) + '/plate.urdf', physicsClientId=self.physId)

        sphereCollisionShapeId = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=ballRadius, physicsClientId=self.physId)
        self.ballId = p.createMultiBody(baseMass=ballMass, baseInertialFramePosition=[ballInertia]*3, 
                                  baseCollisionShapeIndex=sphereCollisionShapeId, baseVisualShapeIndex=-1, 
                                  basePosition = [self.ballPosition[0], self.ballPosition[1], self.ballHeight], physicsClientId=self.physId)

        self.reset()

    # Input - desired angles [-1; 1]
    def step(self, input):

        input = np.clip(input, self.env.action_space_low, self.env.action_space_high)
        self.angleTargets = np.array(input) * self.controlAngleLimit * m.pi / 180

        pos_prop    = 0.01
        vel_prop    = 1
        vel_max     = 2
        force_max   = 3

        p.setJointMotorControl2(bodyUniqueId=self.plateId, jointIndex=0, controlMode=p.POSITION_CONTROL, 
                                positionGain=pos_prop, velocityGain=vel_prop, maxVelocity=vel_max,
                                targetPosition=self.angleTargets[0], force=force_max, physicsClientId=self.physId)

        p.setJointMotorControl2(bodyUniqueId=self.plateId, jointIndex=1, controlMode=p.POSITION_CONTROL,
                                positionGain=pos_prop, velocityGain=vel_prop, maxVelocity=vel_max,
                                targetPosition=self.angleTargets[1], force=force_max, physicsClientId=self.physId)

        p.stepSimulation(physicsClientId=self.physId)
        self.time += self.dt

        self._update_position()

        return self.ballPosition, any(abs(self.ballPosition) > 1.) #self._is_end()

    def _update_position(self):
        ballpos, ballorn = p.getBasePositionAndOrientation(self.ballId, physicsClientId=self.physId)
        
        ls = p.getLinkState(bodyUniqueId=self.plateId, linkIndex=1, physicsClientId=self.physId)
        platePos, plateOrn = ls[0], ls[1]
        
        invPlatePos, invPlateOrn = p.invertTransform(platePos, plateOrn, physicsClientId=self.physId)
        ballPosOnPlate, ballOrnOnPlate = p.multiplyTransforms(invPlatePos, invPlateOrn, ballpos, ballorn, physicsClientId=self.physId)
        ballPosOnPlate = np.array(ballPosOnPlate)

        # [x, y] on plate in range [-1; 1]
        self.ballPosition = ballPosOnPlate[0:2] / self.plateSize
        self.ballHeight = ballpos[2]

    def is_contacted(self):
        result = len(p.getContactPoints(self.ballId, self.plateId, linkIndexB=1, physicsClientId=self.physId))
        # if result > 1:
            # print('Too many balls!')

        return result

    def _is_fallen(self):
        return self.ballHeight < .1

    def _is_end(self):
        return (self._is_fallen() and not self.is_contacted())

    def reset(self):
        self.time           = 0

        if self.randomInitial:
            # self.intial_pos = np.array([((rand.random()-.5) * 2), ((rand.random()-.5) * 2)])
            self.intial_pos = np.array([((rand.random()-.5) * 1), ((rand.random()-.5) * 1)])

        self.ballHeight     = .28
        self.ballPosition   = self.intial_pos * self.plateSize

        # Alpha, Beta
        self.angleTargets = [0, 0]

        p.resetBasePositionAndOrientation(bodyUniqueId=self.ballId, posObj=[self.ballPosition[0], self.ballPosition[1], self.ballHeight], ornObj=[0, 0, 0, 1], physicsClientId=self.physId)

        p.resetJointState(bodyUniqueId=self.plateId, jointIndex=0, targetValue=0, targetVelocity=0, physicsClientId=self.physId)
        p.resetJointState(bodyUniqueId=self.plateId, jointIndex=1, targetValue=0, targetVelocity=0, physicsClientId=self.physId)

        while not self.is_contacted():
            p.stepSimulation(physicsClientId=self.physId)

        self._update_position()

        return self.ballPosition

    def close(self):
        p.disconnect(physicsClientId=self.physId)