import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import time
import constants as c


from world import WORLD
from robot import ROBOT


class SIMULATION:

    def __init__(self, type, solutionID):

        self.directOrGUI = type
        self.simulationID = solutionID

        if type == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT(self.simulationID)

    def Run(self):
        for i in range(0,c.timeLength):
            if(self.directOrGUI == "GUI"):
                time.sleep(c.sleepTime)
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act()

        self.Get_Fitness()

    def Get_Fitness(self):
        self.robot.Get_Fitness(self.simulationID)

    def __del__(self):
        p.disconnect()