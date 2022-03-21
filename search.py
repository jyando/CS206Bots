import os
from hillclimber import HILL_CLIMBER
from parallelHillClimber import PARALLEL_HILL_CLIMBER

# hc = HILL_CLIMBER()
# hc.Evolve()
# hc.Show_Best()

phc = PARALLEL_HILL_CLIMBER()

phc.Evolve()

phc.Show_Best()

#phc.Delete_Files()