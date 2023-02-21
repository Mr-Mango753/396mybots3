from world import WORLD
from robot import ROBOT
from sensor import SENSOR
import time 
import pybullet as p
import pybullet_data 
import pyrosim.pyrosim as pyrosim
import constants as c
import numpy
import random
import os


class SOLUTION:

    def __init__(self, ID) -> None:
        weights = []
        for i in range(c.numSensorNeurons):
            temp = []
            for j in range(c.numMotorNeurons):
                temp.append(numpy.random.rand())
            weights.append(temp)
        weights = numpy.asarray(weights)

        self.weights = weights
        self.weights = self.weights * 2 - 1
        self.fitness = 0
        self.myID = ID
        self.sensors = []
        self.motors = []
        self.links = []
        self.counter = 0

        

    def Set_ID(self):
        self.myID += 1

    # def Evaluate(self, directOrGUI):
    #     self.Create_World()
    #     self.Generate_Body()
    #     self.Generate_Brain()
    #     # os.system("python simulation.py " + directOrGUI + " " + str(self.myID) + " &")
    #     while not os.path.exists("fitness"+str(self.myID)+".txt"):
    #         time.sleep(.01)
    #     f = open("fitness"+str(self.myID)+".txt", "r")
    #     self.fitness = float(f.read())
    #     f.close()
    #     os.system("start /B " + "python simulation.py " + directOrGUI + " " + str(self.myID))
        
    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        if self.myID == 0:
            self.Generate_Body()
        self.Generate_Brain()
        os.system("start /B " + "python simulation.py " + directOrGUI + " " + str(self.myID))


    def Wait_For_Simulation_To_End(self):
        while not os.path.exists(f"fitness{str(self.myID)}.txt"):
            time.sleep(0.01)
        f = open(f"fitness{self.myID}.txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system(f"del fitness{str(self.myID)}.txt")
        
    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        for currentRow in range(c.numSensorNeurons):
            for currentColumn in range(c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow, targetNeuronName=currentColumn+c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])
        for i in self.sensors:
            pyrosim.Send_Sensor_Neuron(name=i, linkName=i)
        for j in self.motors:
            pyrosim.Send_Motor_Neuron(name=j, jointName=j)

        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")
        x = 1
        y = 1 
        z = 1
        starting_position = [0,0,2]
        axis = 0
        for i in range(c.numoflinks):
            randomNum = random.randint(0,1)
            colorArray = ['<color rgba="0 1 0 1"/>', '<color rgba="0 0 1 1"/>']
            colorNameArray = ['<material name="Green">', '<material name="Blue">']
            color = colorArray[randomNum]
            colorname = colorNameArray[randomNum]
            pyrosim.Send_Cube(name=f'link{i}', pos=starting_position , size=[x,y,z], color = color, colorname = colorname)
            lastAxis = axis
            axisArray = ['x', 'y', 'z']
            randomNum2 = random.randint(0,2)
            axis = axisArray[randomNum2]
            randomNum3 = random.randint(0,2)
            jointAxisArray = ["1 0 0", "0 1 0", "0 0 1"]
            jointAxis = jointAxisArray[randomNum3]
            self.Generate_Joints(axis, lastAxis, i, x, y, z, starting_position, jointAxis)
            
            x=random.uniform(0, 1)
            y=random.uniform(0, 1)
            z=random.uniform(0, 1)
            
            if axis == 'x':
                starting_position = [-x/2, 0, 0]
            if axis == 'y':
                starting_position = [0, -y/2, 0]
            if axis == 'z':
                starting_position = [0, 0, -z/2]
        pyrosim.End()
       

    def Generate_Joints(self, axis, lastAxis, i, x, y, z, position, jointAxis):
        maxLinks = c.numoflinks-1
        stringI = str(i)
        stringIPlus = str(i + 1)
        if axis == 'x':
            if i == 0:
                pyrosim.Send_Joint(name="link"+stringI+"_"+"link"+stringIPlus,parent="link"+stringI,child="link"+stringIPlus,type="revolute",position=[-x/2,0,position[2]],jointAxis=jointAxis)
            elif i < maxLinks:
                if lastAxis == 'y':
                    pyrosim.Send_Joint(name="link"+stringI+"_"+"link"+stringIPlus,parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [-x/2, -y/2, 0], jointAxis=jointAxis)
                elif lastAxis == 'z':
                    pyrosim.Send_Joint(name="link"+stringI+"_"+"link"+stringIPlus,parent="link"+stringI,child="link"+stringIPlus,type="revolute",position=[-x/2,0,-z/2],jointAxis=jointAxis)
                else:
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [-x, 0, 0], jointAxis=jointAxis)
        elif axis == 'y':
            if i == 0:
                pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [0, -x/2, position[2]], jointAxis=jointAxis)
            elif i < maxLinks:
                if lastAxis == 'x':
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [-x/2, -y/2, 0], jointAxis=jointAxis)
                elif lastAxis == 'z':
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [0, -y/2, -z/2], jointAxis=jointAxis)
                else:                    
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [0, -y, 0], jointAxis=jointAxis)
        else:
            if i == 0:
                pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [0, 0, z-z/2], jointAxis=jointAxis)
            elif i < maxLinks:
                if lastAxis == 'x':
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [-x/2, 0, -z/2], jointAxis=jointAxis)
                elif lastAxis == 'y':
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [0, -y/2, -z/2], jointAxis=jointAxis)
                else:
                    pyrosim.Send_Joint( name = "link" + stringI + "_" + "link" + stringIPlus , parent= "link" + stringI , child = "link" + stringIPlus , type = "revolute", position = [0, 0, -z], jointAxis=jointAxis)
            
    def Set_ID(self, ID):
        self.myID = ID

    def Mutate(self):
        randRow = random.randint(0,c.numSensorNeurons-1)
        randColumn = random.randint(0,c.numMotorNeurons-1)
        self.weights[randRow, randColumn] = random.random() * 2 - 1
