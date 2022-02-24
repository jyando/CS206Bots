import constants as c
import numpy
import pyrosim.pyrosim as pyrosim
import pybullet as p
import os

class MOTOR:

    def __init__(self, jointName):
        self.jointName= jointName
        print(self.jointName)

    def Set_Value(self, robotId, desiredAngle):
        self.robotId = robotId
        pyrosim.Set_Motor_For_Joint(
            bodyIndex=self.robotId,
            jointName=self.jointName,
            controlMode=p.POSITION_CONTROL,
            targetPosition= desiredAngle,
            maxForce=c.maxForce)
