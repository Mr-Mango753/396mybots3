import numpy
import numpy
import constants as c
import pyrosim.pyrosim as pyrosim


class SENSOR:

    def __init__(self, linkName):
        self.sensors = {}
        self.values = numpy.zeros(c.vectorSize)
        self.linkName = linkName

    def Get_Value(self, i):
        self.values[i] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    def Save_Values(self, i):
        numpy.save('data\sensorvalsnew.npy', self.values)