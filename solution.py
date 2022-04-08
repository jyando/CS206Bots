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
        pyrosim.Send_Joint(name="Torso_BackLeftLeg", parent="Torso", child="BackLeftLeg", type="revolute", position=[-0.25, -0.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeftLeg", pos=[0, -0.5, 0], size=[0.2,1,0.2])
        pyrosim.Send_Joint(name="Torso_BackRightLeg", parent="Torso", child="BackRightLeg", type="revolute",
                           position=[0.25, -0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BackRightLeg", pos=[0, -0.5, 0], size=[0.2, 1, 0.2])
        pyrosim.Send_Joint(name="Torso_FrontLeftLeg", parent="Torso", child="FrontLeftLeg", type="revolute", position=[-0.25, 0.5, 1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeftLeg", pos=[0, 0.5, 0], size=[0.2,1,0.2])

        pyrosim.Send_Joint(name="Torso_FrontRightLeg", parent="Torso", child="FrontRightLeg", type="revolute",
                           position=[0.25, 0.5, 1], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="FrontRightLeg", pos=[0, 0.5, 0], size=[0.2, 1, 0.2])

        pyrosim.Send_Joint(name="Torso_LeftFrontLeg", parent="Torso", child="LeftFrontLeg", type="revolute", position=[-0.5, 0.25, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftFrontLeg", pos=[-0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_LeftBackLeg", parent="Torso", child="LeftBackLeg", type="revolute",
                           position=[-0.5, -0.25, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LeftBackLeg", pos=[-0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_RightFrontLeg", parent="Torso", child="RightFrontLeg", type="revolute", position=[0.5, 0.25, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightFrontLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_RightBackLeg", parent="Torso", child="RightBackLeg", type="revolute",
                           position=[0.5, -0.25, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="RightBackLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])

        #  Add the lower legs now
        pyrosim.Send_Joint(name="FrontLeg_LowerFrontLeftLeg", parent="FrontLeftLeg", child="LowerFrontLeftLeg", type="revolute",
                           position=[0, 1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerFrontLeftLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="FrontLeg_LowerFrontRightLeg", parent="FrontRightLeg", child="LowerFrontRightLeg",
                           type="revolute",
                           position=[0, 1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerFrontRightLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="BackLeg_LowerBackLeftLeg", parent="BackLeftLeg", child="LowerBackLeftLeg", type="revolute",
                           position=[0, -1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerBackLeftLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="RightLeg_LowerRightFrontLeg", parent="RightFrontLeg", child="LowerRightFrontLeg", type="revolute",
                           position=[1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerRightFrontLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])
        pyrosim.Send_Joint(name="RightLeg_LowerRightBackLeg", parent="RightBackLeg", child="LowerRightBackLeg",
                           type="revolute",
                           position=[1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerRightBackLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="LeftLeg_LowerLeftFrontLeg", parent="LeftFrontLeg", child="LowerLeftFrontLeg", type="revolute",
                           position=[-1,0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerLeftFrontLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="LeftLeg_LowerLeftBackLeg", parent="LeftBackLeg", child="LowerLeftBackLeg",
                           type="revolute",
                           position=[-1, 0, 0], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="LowerLeftBackLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name="BackLeg_LowerBackRightLeg", parent="BackRightLeg", child="LowerBackRightLeg", type="revolute",
                           position=[0, -1, 0], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="LowerBackRightLeg", pos=[0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.End()

    def Create_Brain(self):
        self.brainNameFile = ("brain" + str(self.myID) + ".nndf")
        #self.brainNameFile = ("brain.nndf")
        pyrosim.Start_NeuralNetwork(self.brainNameFile)

        pyrosim.Send_Sensor_Neuron(name=0, linkName="LowerBackLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="LowerFrontLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="LowerLeftFrontLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="LowerRightFrontLeg")

        pyrosim.Send_Sensor_Neuron(name=4, linkName="LowerBackRightLeg")
        pyrosim.Send_Sensor_Neuron(name=5, linkName="LowerFrontRightLeg")
        pyrosim.Send_Sensor_Neuron(name=6, linkName="LowerLeftBackLeg")
        pyrosim.Send_Sensor_Neuron(name=7, linkName="LowerRightBackLeg")

        pyrosim.Send_Motor_Neuron(name=8, jointName="Torso_BackLeftLeg")
        pyrosim.Send_Motor_Neuron(name=9, jointName="Torso_FrontLeftLeg")
        pyrosim.Send_Motor_Neuron(name=10, jointName="Torso_LeftFrontLeg")
        pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_RightFrontLeg")
        pyrosim.Send_Motor_Neuron(name=12, jointName="FrontLeg_LowerFrontLeftLeg")
        pyrosim.Send_Motor_Neuron(name=13, jointName="BackLeg_LowerBackLeftLeg")
        pyrosim.Send_Motor_Neuron(name=14, jointName="LeftLeg_LowerLeftFrontLeg")
        pyrosim.Send_Motor_Neuron(name=15, jointName="RightLeg_LowerRightFrontLeg")

        pyrosim.Send_Motor_Neuron(name=16, jointName="Torso_BackRightLeg")
        pyrosim.Send_Motor_Neuron(name=17, jointName="BackLeg_LowerBackRightLeg")
        pyrosim.Send_Motor_Neuron(name=18, jointName="Torso_FrontRightLeg")
        pyrosim.Send_Motor_Neuron(name=19, jointName="FrontLeg_LowerFrontRightLeg")

        pyrosim.Send_Motor_Neuron(name=20, jointName="Torso_LeftBackLeg")
        pyrosim.Send_Motor_Neuron(name=21, jointName="LeftLeg_LowerLeftBackLeg")
        pyrosim.Send_Motor_Neuron(name=22, jointName="Torso_RightBackLeg")
        pyrosim.Send_Motor_Neuron(name=23, jointName="RightLeg_LowerRightBackLeg")

        for currentRow in range(c.numSensorNeurons):

            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+c.numSensorNeurons, weight=(self.weights[currentRow][currentColumn]))

        pyrosim.End()

    def Set_ID(self, ID):
        self.myID = ID