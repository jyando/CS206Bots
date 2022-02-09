import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy
import os
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = numpy.zeros(1000)
frontLegSensorValues = numpy.zeros(1000)

targetAngles = (numpy.pi/4.0)*numpy.sin(numpy.linspace(0, 2 * numpy.pi, 1000))

pathSin = 'data/sinFunction.npy'
numpy.save(pathSin, targetAngles)

for i in range(0,1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotId,
        jointName="Torso_BackLeg",
        controlMode=p.POSITION_CONTROL,
        targetPosition= targetAngles[i],# (random.random()*numpy.pi)-(numpy.pi/2.0),
        maxForce=35)

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotId,
        jointName="Torso_FrontLeg",
        controlMode=p.POSITION_CONTROL,
        targetPosition=targetAngles[i], #(random.random()*numpy.pi)-(numpy.pi/2.0),
        maxForce=35)

    time.sleep(1/60)
    print(i)


pathBack = 'data/backLegData.npy'
pathFront = 'data/frontLegData.npy'
numpy.save(pathBack, backLegSensorValues)
numpy.save(pathFront, frontLegSensorValues)


# print(frontLegSensorValues)

p.disconnect()