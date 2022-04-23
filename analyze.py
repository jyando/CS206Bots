import numpy
import pandas as pd
import matplotlib.pyplot

filenameA = 'testA.csv'
filenameB = 'testB.csv'

testAData = pd.read_csv(filenameA)
testBData = pd.read_csv(filenameB)

testAMeans = testAData.mean(axis=0)
testBMeans = testBData.mean(axis=0)

matplotlib.pyplot.plot(testAMeans, label = "Mean Fitness Values of Test A Generations", linewidth = 2)
matplotlib.pyplot.plot(testBMeans, label = "Mean Fitness Values of Test B Generations")
matplotlib.pyplot.legend()
matplotlib.pyplot.show()

