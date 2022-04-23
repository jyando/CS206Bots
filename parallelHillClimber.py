from solution import SOLUTION
import constants as c
import copy
import os
import numpy
import pandas
import csv
import constants as c
from pyrosim.neuralNetwork import NEURAL_NETWORK

class PARALLEL_HILL_CLIMBER:

    def __init__(self):

        #for i in range((c.populationSize + 1) * (c.numberOfGenerations + 1) + 1):
        for i in range(c.populationSize + c.numberOfGenerations):
            os.system("del brain" + str(i) + ".nndf")
            os.system("del fitness" + str(i) + ".txt")

        self.parents = {}

        self.nextAvailableID = 0

        self.data = numpy.zeros([c.populationSize, c.numberOfGenerations])
        self.row = 0
        self.column = 0

        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1


    def Evolve(self):

        self.Evaluate(self.parents)

        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()
            self.column += 1

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()
        pass

    def Spawn(self):

        self.children = {}

        for i in range(len(self.parents)):
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
            self.children[i] = SOLUTION(self.nextAvailableID)



    def Mutate(self):
        for i in range(len(self.children)):
            self.children[i].Mutate()

    def Select(self):

        self.row = 0
        for i in range(len(self.parents)):
            if(self.children[i].fitness > self.parents[i].fitness):
                self.parents[i] = self.children[i]

            self.data[self.row][self.column] = self.parents[i].fitness
            self.row += 1

    def Print(self):

        for i in range(len(self.parents)):
            print(" ")
            print("Parent Fitness Value: ", self.parents[i].fitness, " Child Fitness Value: ", self.children[i].fitness)
            print("")


    def Show_Best(self):

        #print(self.data)

        self.bestKey = 0
        for i in range (len(self.parents)):
            if self.parents[i].fitness > self.parents[self.bestKey].fitness:
                self.bestKey = i

        self.parents[self.bestKey].Start_Simulation("GUI")


    def Evaluate(self, solutions):

        for i in range(len(solutions)):
            solutions[i].Start_Simulation("DIRECT")

        for i in range(len(solutions)):
            solutions[i].Wait_For_Simulation_To_End()

    def WriteOut(self):
        filename = "testA.csv"

        self.dataPd = pandas.DataFrame(self.data)

        print(self.dataPd)
        self.dataPd.to_csv(filename, mode='a', index=False, header=False)
        # writing to csv file
        # with open(filename, 'w') as csvfile:
        #     # creating a csv writer object
        #     csvwriter = csv.writer(csvfile)
        #     csvwriter.writerow(self.dataPd[1])
