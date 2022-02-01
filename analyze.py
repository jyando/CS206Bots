import numpy
import matplotlib.pyplot

filenameBack = 'data/backLegData.npy'
filenameFront = 'data/frontLegData.npy'

backLegSensorValues = numpy.load(filenameBack)
frontLegSensorValues = numpy.load(filenameFront)

matplotlib.pyplot.plot(backLegSensorValues, label = "Back Leg Sensor", linewidth = 2)
matplotlib.pyplot.plot(frontLegSensorValues, label = " Front Leg Sensor")
matplotlib.pyplot.legend()
matplotlib.pyplot.show()