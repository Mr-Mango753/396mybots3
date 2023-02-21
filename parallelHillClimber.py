from solution import SOLUTION
import constants as c
import copy
import os

class PARALLELHILLCLIMBER:
    def __init__(self) -> None:
        self.parents = {}
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")

    def Spawn(self):
        self.children = {}
        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for i in self.children:
            self.children[i].Mutate()

    def Select(self):
        # print(self.parents.fitness, self.children.fitness)
        for i in self.parents:
            if self.parents[i].fitness > self.children[i].fitness:
                self.parents[i] = self.children[i]

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Evaluate(self, results):
        for i in results:
            results[i].Start_Simulation("DIRECT")
        for i in results:
            results[i].Wait_For_Simulation_To_End()

    def Show_Best(self):
        mostFit = self.parents[0].fitness
        index = 0
        for i in self.parents:
            if self.parents[i].fitness < mostFit:
                mostFit = self.parents[i].fitness
                index = i
        self.parents[index].Start_Simulation("GUI")

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Print(self):
        print("\n")
        for i in self.parents:
            print(f"Parent: {self.parents[i].fitness} Child: {self.children[i].fitness}")
        print("\n")
