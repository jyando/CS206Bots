from simulation import SIMULATION
import os
import sys

#os.system("py simulate.py DIRECT")

directOrGUI = sys.argv[1]


simulation = SIMULATION(directOrGUI)
simulation.Run()