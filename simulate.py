from simulation import SIMULATION
import os
import sys

#os.system("py simulate.py DIRECT")

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]


simulation = SIMULATION(directOrGUI, solutionID)
simulation.Run()