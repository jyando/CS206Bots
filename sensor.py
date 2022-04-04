import numpy

import constants as c
import pyrosim.pyrosim as pyrosim
import os

class SENSOR:

    def __init__(self, linkName):
        self.linkName= linkName
        self.values = numpy.zeros(1000)

    def Get_Value(self, timeStep):
        self.timeStep = timeStep
        self.values[self.timeStep] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        if (self.timeStep == c.timeLength):
            print(self.values)

    def getSumValues(self):
        return numpy.sum(self.values)

    def Save_Values(self):
        self.pathSaveSensors = 'data/sensorsData.npy'
        numpy.save(self.pathSaveSensors, self.values)