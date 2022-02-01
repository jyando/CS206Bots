import numpy
import matplotlib.pyplot

filename = 'data/backLegData.npy'

backLegSensorValues = numpy.load(filename)


matplotlib.pyplot.plot(backLegSensorValues)
matplotlib.pyplot.show()