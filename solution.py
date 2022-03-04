import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import os

import numpy

class SOLUTION:

    def __init__(self):
        self.weights = numpy.zeros((3,2))

        for i in range(self.weights[0].size):
            for j in range(self.weights[:,0].size):
            #for j in range(3):
                self.weights[j,i] = numpy.random.rand()

        self.weights = (2*self.weights) - 1

    def Evaluate(self):
            self.Create_World()
            self.Create_Body()
            self.Create_Brain()
            os.system("py simulate.py")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        x = -2.5
        y = 2.5
        z = 0.5
        pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[c.length, c.width, c.height])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[1.5, 0, 1.5], size=[c.length, c.width, c.height])
        pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[1, 0, 1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-.5, 0, -.5], size=[c.length, c.width, c.height])
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[2, 0, 1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[.5, 0, -.5], size=[c.length, c.width, c.height])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain.nndf")

        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")

        for currentRow in range(3):

            for currentColumn in range(2):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+3, weight=(self.weights[currentRow][currentColumn]))

        pyrosim.End()