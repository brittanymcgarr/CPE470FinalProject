################################################################################
# main.py                                                                      #
#                                                                              #
# The main entry point for the application. Sends how many desired rounds and  #
# target data to the robot controllers.                                        #
#                                                                              #
# Project #2: Mobile Robot Path Planning Using Artificial Potential Field      #
# CPE 470 Fall 2016                                                            #
# Brittany McGarr                                                              #
################################################################################

import robotData


def testCases():
    # Create the data instances
    dataLine = robotData.RobotData()
    dataCircle = robotData.RobotData()
    dataSine = robotData.RobotData(300)

    # Run the simulation without noise
    print "Running simulator for Linear Path..."
    dataLine.runSimulation()

    print "Running simulator for Sine Path..."
    dataSine.runSimulation(path="sin")

    # Run the simulation with noise
    print "Running simulator for Linear Path WITH NOISE..."
    dataLine.runSimulation(noise=True)

    print "Running simulator for the Sine Path WITH NOISE..."
    dataSine.runSimulation(path="sin", noise=True)

    # Run the simulation on the circular paths
    print "Running the simulator for the Circular paths..."
    dataCircle.runSimulation(path="circle")
    dataCircle.runSimulation(path="circle", noise=True)


testCases()
