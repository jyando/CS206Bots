import constants as c
import numpy
import pyrosim.pyrosim as pyrosim
import pybullet as p
import os

class MOTOR:

    def __init__(self, jointName):
        self.jointName= jointName
        print(self.jointName)
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        if(self.jointName == "Torso_BackLeg"):
            self.amplitude = c.amplitudeBL
            self.frequency = 4
            self.offset = c.phaseOffSetBL
        else:
            self.amplitude = c.amplitudeFL
            self.frequency = c.frequencyFL
            self.offset = c.phaseOffSetFL

        self.x = numpy.linspace(0, 2*(numpy.pi), 1000)

        self.motorValues = numpy.zeros(1000)
        for i in range(1000):
            self.motorValues[i] = self.amplitude *(numpy.sin((self.frequency*self.x[i])+self.offset))

    def Set_Value(self, robotId, timeStep):
        self.timeStep = timeStep
        self.robotId = robotId
        pyrosim.Set_Motor_For_Joint(
            bodyIndex=self.robotId,
            jointName=self.jointName,
            controlMode=p.POSITION_CONTROL,
            targetPosition=self.motorValues[self.timeStep],
            maxForce=c.maxForce)

    def Save_Values(self):
        self.pathSaveMotors = 'data/motorsData.npy'
        numpy.save(self.pathSaveMotors, self.motorValues)