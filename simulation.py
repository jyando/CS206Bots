import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import time
import constants as c


from world import WORLD
from robot import ROBOT


class SIMULATION:

    def __init__(self, type):

        print(type)

        if type == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)

        self.world = WORLD()
        self.robot = ROBOT()

    def Run(self):
        for i in range(0,c.timeLength):
            time.sleep(c.sleepTime)
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act()

        self.Get_Fitness()

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()