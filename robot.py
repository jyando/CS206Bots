import pybullet as p
import pyrosim.pyrosim as pyrosim
import os
from pyrosim.neuralNetwork import NEURAL_NETWORK

from sensor import SENSOR
from motor import MOTOR

class ROBOT:

    def __init__(self, solutionID):
        self.robotId = p.loadURDF("body.urdf")

        self.nn = NEURAL_NETWORK("brain" + str(solutionID) + ".nndf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        os.system("del brain" + solutionID + ".nndf")
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Prepare_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName]= MOTOR(jointName)

    def Sense(self, timeStep):
        self.timeStep = timeStep
        for i in self.sensors:
            self.sensorI = self.sensors[i]
            self.sensorI.Get_Value(self.timeStep)

    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                self.motors[jointName].Set_Value(self.robotId, desiredAngle)

    def Get_Fitness(self, ID):
        self.stateOfLinkZero = p.getLinkState(self.robotId,0)
        self.positionOfLinkZero = self.stateOfLinkZero[0]
        self.xCoordinateOfLinkZero = self.positionOfLinkZero[0]

        with open('tmp' + ID + '.txt', 'w') as f:
            f.write(str(self.xCoordinateOfLinkZero))

        os.system("rename tmp"+str(ID)+".txt " + "fitness"+str(ID)+".txt")

