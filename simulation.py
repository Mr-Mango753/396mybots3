import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy
import random
from simulation1 import SIMULATION
import sys

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]
simulation = SIMULATION(directOrGUI, solutionID)
simulation.Run()
simulation.Get_Fitness()
# sys.setrecursionlimit(10**6)


# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.setGravity(0,0,-25.8)
# planeId = p.loadURDF("plane.urdf")
# p.loadSDF("world.sdf")
# robotId = p.loadURDF("body.urdf")
# pyrosim.Prepare_To_Simulate(robotId)
# # robot = pyrosim.Prepare_To_Simulate(robotId)
# backLegSensorValues = numpy.zeros(1000)
# frontLegSensorValues = numpy.zeros(1000)
# amplitudeBackLeg = (numpy.pi)/4
# frequencyBackleg = 7
# phaseOffsetBackleg = (numpy.pi)/4

# amplitudeFrontLeg = (numpy.pi)/4
# frequencyFrontLeg = 3
# phaseOffsetFrontLeg = (numpy.pi)/4

# targetAnglesBackLeg = numpy.linspace(0, 2 * numpy.pi, 1000)
# targetAnglesFrontLeg = numpy.linspace(0, 2 * numpy.pi, 1000)

# for i in range(1000):
#     backLegTouch = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     # print(backLegTouch)
#     backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
#     p.stepSimulation()
#     time.sleep(1/240)
#     print(backLegSensorValues)  
#     numpy.save("data/backLegSensorValues", backLegSensorValues, allow_pickle=True, fix_imports=True)
#     numpy.save("data/frontLegSensorValues", frontLegSensorValues, allow_pickle=True, fix_imports=True)
#     targetAnglesBackLeg[i] = amplitudeBackLeg * numpy.sin(frequencyBackleg * targetAnglesBackLeg[i] + phaseOffsetBackleg)
#     targetAnglesFrontLeg[i] = amplitudeFrontLeg * numpy.sin(frequencyFrontLeg * targetAnglesFrontLeg[i] + phaseOffsetFrontLeg)
#     numpy.save("data/targetAnglesValuesBackLeg", targetAnglesBackLeg, allow_pickle=True, fix_imports=True)
#     numpy.save("data/targetAnglesValuesFrongLeg", targetAnglesFrontLeg, allow_pickle=True, fix_imports=True)
#     pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = "Torso_BackLeg", controlMode = p.POSITION_CONTROL, targetPosition = targetAnglesBackLeg[i], maxForce = 200)
#     pyrosim.Set_Motor_For_Joint(bodyIndex = robotId, jointName = "Torso_FrontLeg", controlMode = p.POSITION_CONTROL, targetPosition = targetAnglesFrontLeg[i], maxForce = 200)
# p.disconnect()
