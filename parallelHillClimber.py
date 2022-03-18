from solution import SOLUTION
import constants as c
import copy
import os
import constants as c

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        self.parents = {}

        self.nextAvailableID = 0

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1


    def Evolve(self):

        for i in range(len(self.parents)):
            self.parents[i].Evaluate("GUI")

        # self.parent.Evaluate("GUI")
        #
        # for currentGeneration in range(c.numberOfGenerations):
        #     self.Evolve_For_One_Generation()

        pass

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Print()
        self.Select()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)
        self.child.Set_ID(self.nextAvailableID)
        self.nextAvailableID += 1

    def Mutate(self):
        self.child.Mutate()

    def Select(self):

        if(self.child.fitness > self.parent.fitness):
            self.parent.fitness = self.child.fitness

    def Print(self):
        print("\nParent Fitness Value: ", self.parent.fitness, " Child Fitness Value: ", self.child.fitness)

    def Show_Best(self):
        # os.system("py simulate.py GUI")
        pass