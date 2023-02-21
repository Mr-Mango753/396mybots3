import os

from parallelHillClimber import PARALLELHILLCLIMBER

phc = PARALLELHILLCLIMBER()
phc.Evolve()
phc.Show_Best()

# for i in range(5):
#     os.system("python3 generate.py")
#     os.system("python3 simulation.py")