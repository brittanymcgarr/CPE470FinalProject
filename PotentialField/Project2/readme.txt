README.txt

To run the test data on Linux and Unix platforms, be sure NumPy and PyPlot libraries are installed and working. Navigate to the directory housing the program and run main.py in your Python interpreter.

user$ python main.py

Using the Simulator for Individual Tests:
The RobotData class can also be adjusted to run the simulation for individual tests. Define the number of iterations desired in the creation of an instanced object and offer arguments for the "sin," "circle," line and noisy boolean.

e.g. 

# Running 250 iterations of the sine-wave test with noise

import robotData

data = robotData.RobotData(250)

data.runSimulation(path="sin", noise=True)


