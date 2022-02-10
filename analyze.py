import numpy
import matplotlib.pyplot

filenameBack = 'data/backLegData.npy'
filenameFront = 'data/frontLegData.npy'
filenameSinFL = 'data/sinFunctionFL.npy'
filenameSinBL = 'data/sinFunctionBL.npy'

# backLegSensorValues = numpy.load(filenameBack)
# frontLegSensorValues = numpy.load(filenameFront)
sinValuesFL = numpy.load(filenameSinFL)
sinValuesBL = numpy.load(filenameSinBL)


#matplotlib.pyplot.plot(backLegSensorValues, label = "Back Leg Sensor", linewidth = 2)
#matplotlib.pyplot.plot(frontLegSensorValues, label = " Front Leg Sensor")
matplotlib.pyplot.plot(sinValuesFL, label = " Front Leg Sensor", linewidth = 5)
matplotlib.pyplot.plot(sinValuesBL, label = " Back Leg Sensor")
matplotlib.pyplot.legend()
matplotlib.pyplot.show()