import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import os
import random
import time

import numpy

class SOLUTION:

    def __init__(self, ID):
        self.weights = numpy.zeros((c.numSensorNeurons, c.numMotorNeurons))

        self.myID = ID

        for i in range(self.weights[0].size):
            for j in range(self.weights[:,0].size):
                self.weights[j,1] = numpy.random.rand()

        self.weights = (2*self.weights) - 1

    def Evaluate(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

        os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID))

        self.fitnessFileName = ('fitness' + str(self.myID) + '.txt')

        while not os.path.exists(self.fitnessFileName):
            time.sleep(0.01)

        f = open(self.fitnessFileName, 'r')
        self.fitness = float(f.readline())
        print(self.fitness)
        f.close()

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

        os.system("start /B py simulate.py " + directOrGUI + " " + str(self.myID))

    def Wait_For_Simulation_To_End(self):
        self.fitnessFileName = ('fitness' + str(self.myID) + '.txt')

        while not os.path.exists(self.fitnessFileName):
            time.sleep(0.01)

        f = open(self.fitnessFileName, 'r')
        self.fitness = float(f.readline())
        f.close()
        os.system("del " + str(self.fitnessFileName))
        #print(self.fitness)

    def Mutate(self):
        randomRow = random.randint(0,c.numSensorNeurons - 1)
        randomColumn = random.randint(0,c.numMotorNeurons - 1)

        self.weights[randomRow, randomColumn] = (2*numpy.random.rand()) - 1

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        x = -2.5
        y = 2.5
        z = 0.5
        pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[c.length, c.width, c.height])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1], size=[c.length, c.width, c.height])
        pyrosim.Send_Joint(name="Torso_BackLeftLeg", parent="Torso", child="BackLeftLeg", type="revolute", position=[0, -0.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeftLeg", pos=[-0.25, -0.5, 0], size=[0.2,1,0.2])
        pyrosim.Send_Joint(name="Torso_BackRightLeg", parent="Torso", child="BackRightLeg", type="revolute",
                           position=[0, -0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackRightLeg", pos=[0.25, -0.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[0, 0.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0], size=[0.2,1,0.2])
        pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg", type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg", type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])

        #  Add the lower legs now
        pyrosim.Send_Joint(name="FrontLeg_LowerFrontLeg", parent="FrontLeg", child="LowerFrontLeg", type="revolute",
                           position=[0, 1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerFrontLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="BackLeg_LowerBackLeftLeg", parent="BackLeftLeg", child="LowerBackLeftLeg", type="revolute",
                           position=[0, -1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerBackLeftLeg", pos=[-0.25, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="RightLeg_LowerRightLeg", parent="RightLeg", child="LowerRightLeg", type="revolute",
                           position=[1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerRightLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="LeftLeg_LowerLeftLeg", parent="LeftLeg", child="LowerLeftLeg", type="revolute",
                           position=[-1,0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerLeftLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="BackLeg_LowerBackRightLeg", parent="BackRightLeg", child="LowerBackRightLeg", type="revolute",
                           position=[0, -1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerBackRightLeg", pos=[0.25, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.End()

    def Create_Brain(self):
        self.brainNameFile = ("brain" + str(self.myID) + ".nndf")
        #self.brainNameFile = ("brain.nndf")
        pyrosim.Start_NeuralNetwork(self.brainNameFile)

        pyrosim.Send_Sensor_Neuron(name=0, linkName="LowerBackLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="LowerFrontLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="LowerLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="LowerRightLeg")

        pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_BackLeftLeg")
        pyrosim.Send_Motor_Neuron(name=5, jointName="Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name=6, jointName="Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name=7, jointName="Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name=8, jointName="FrontLeg_LowerFrontLeg")
        pyrosim.Send_Motor_Neuron(name=9, jointName="BackLeg_LowerBackLeftLeg")
        pyrosim.Send_Motor_Neuron(name=10, jointName="LeftLeg_LowerLeftLeg")
        pyrosim.Send_Motor_Neuron(name=11, jointName="RightLeg_LowerRightLeg")

        pyrosim.Send_Motor_Neuron(name=12, jointName="Torso_BackRightLeg")
        pyrosim.Send_Motor_Neuron(name=13, jointName="BackLeg_LowerBackRightLeg")

        for currentRow in range(c.numSensorNeurons):

            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+3, weight=(self.weights[currentRow][currentColumn]))

        pyrosim.End()

    def Set_ID(self, ID):
        self.myID = ID