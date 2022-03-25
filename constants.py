import numpy

# Front Legs
amplitudeFL = numpy.pi/4
frequencyFL = 50
phaseOffSetFL = 0#-(3/2) * numpy.pi

# Back Legs
amplitudeBL = numpy.pi/4
frequencyBL = 15
phaseOffSetBL = 0#numpy.pi/4

maxForce = 100
sleepTime = 1/100
timeLength = 750

# Cube size
length = 1
width = 1
height = 1

# Number of generations
numberOfGenerations = 10

# Population Size
populationSize = 10

# Dimensions of Weights
numSensorNeurons = 4
numMotorNeurons = 9

motorJointRange = 0.2