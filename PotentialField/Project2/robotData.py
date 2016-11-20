################################################################################
# robotData.py                                                                 #
#                                                                              #
# The main class for the robot (and target 'bot). Maintains the current and    #
# predictive measurements.                                                     #
#                                                                              #
# Project #2: Mobile Robot Path Planning Using Artificial Potential Field      #
# CPE 470 Fall 2016                                                            #
# Brittany McGarr                                                              #
################################################################################

import math
import numpy
import numpy.linalg
import matplotlib.pyplot as plot
import matplotlib.patches as patches


class RobotData:

    # Constructor and initialization of data
    def __init__(self, iterate=100):
        self.deltaTime = 0.05
        self.attractFactor = 8.5
        self.iterations = iterate

        # Target Data
        self.targetMaxVelocity = 1.2
        self.targetPositions = numpy.zeros((self.iterations, 2))
        self.targetHeadings = numpy.zeros((self.iterations, 1))

        # Robot Data
        self.robotMaxVelocity = 50
        self.robotPositions = numpy.zeros((self.iterations, 2))
        self.robotVelocities = numpy.zeros((self.iterations, 1))
        self.robotHeadings = numpy.zeros((self.iterations, 1))

        # Initialize relative states between robot and target
        self.relativePositions = numpy.zeros((self.iterations, 2))
        self.relativePosAbsolute = numpy.zeros((self.iterations, 2))
        self.relativeVelocities = numpy.zeros((self.iterations, 2))
        self.relativeHeadings = numpy.zeros((self.iterations, 1))

        # Noise relative parameters
        self.noiseMean = 0.5
        self.noiseSTD = 0.1

        # Initialize the target's values
        self.targetPositions[0, 0] = 60 - 15 * numpy.cos(0.0)
        self.targetPositions[0, 1] = 30 + 15 * numpy.sin(0.0)

    def setTargetPositionCircular(self, index, timestep):
        self.targetPositions[index, 0] = 60 - 15 * numpy.cos(timestep)
        self.targetPositions[index, 1] = 30 + 15 * numpy.sin(timestep)

    def setTargetPositionLinear(self, index):
        self.targetPositions[index, 0] = self.targetPositions[index - 1, 0] + self.targetMaxVelocity * self.deltaTime
        self.targetPositions[index, 1] = self.targetPositions[index - 1, 1] + self.targetMaxVelocity * self.deltaTime

    def setTargetPositionSine(self, index, timestep):
        self.targetPositions[index, 0] = self.targetPositions[index - 1, 0] + self.targetMaxVelocity * self.deltaTime
        self.targetPositions[index, 1] = self.targetPositions[index - 1, 1] + self.targetMaxVelocity * numpy.sin(timestep)

    def runSimulation(self, path="", noise=False):
        timestep = 0.0

        targetPath = path

        if path == "":
            targetPath = "line"

        if noise:
            targetPath += "Noisy"

        # Run the simulation for each time step
        for index in range(1, self.iterations):
            timestep += self.deltaTime

            # Advance the time step
            # Find the new target position based on the time step
            if path == "circle":
                self.setTargetPositionCircular(index=index, timestep=timestep)
            elif path == "sin":
                self.setTargetPositionSine(index=index, timestep=timestep)
            else:
                self.setTargetPositionLinear(index=index)

            if noise:
                self.addNoise(index)

            targetX = self.targetPositions[index, 0]
            targetY = self.targetPositions[index, 1]

            # Set the target heading
            self.targetHeadings[index, 0] = math.atan2(targetY, targetX)

            # Compute the relative position of the virtual target and robot
            self.relativePositions[index, 0] = targetX - self.robotPositions[index-1, 0]
            self.relativePositions[index, 1] = targetY - self.robotPositions[index-1, 1]

            relativePositionX = self.relativePositions[index, 0]
            relativePositionY = self.relativePositions[index, 1]

            # Store the absolute distance values for plotting
            self.relativePosAbsolute[index, 1] = numpy.max([math.fabs(relativePositionX), math.fabs(relativePositionY)])
            self.relativePosAbsolute[index, 0] = index

            # Compute the relative heading of the virtual target and robot
            self.relativeHeadings[index, 0] = math.atan2(relativePositionY, relativePositionX)

            relativeHeading = self.relativeHeadings[index, 0]

            # Control and record the velocity and heading of the robot
            targetPosMag = numpy.linalg.norm(self.targetPositions[index])
            targetPosMagSqr = targetPosMag * targetPosMag
            robotPosMag = numpy.linalg.norm(self.robotPositions[index-1])
            relativePosMag = numpy.linalg.norm(self.relativePositions[index])
            relativePosMagSqr = relativePosMag * relativePosMag

            self.robotVelocities[index, 0] = math.sqrt((self.targetMaxVelocity * self.targetMaxVelocity) +
                                                       2 * self.attractFactor * relativePosMag *
                                                       self.targetMaxVelocity *
                                                       math.fabs(numpy.cos(self.targetHeadings[index, 0] -
                                                                                          relativeHeading)) +
                                                       (self.attractFactor * self.attractFactor) * relativePosMagSqr)

            # Maintain a maximal velocity
            if self.robotVelocities[index, 0] > self.robotMaxVelocity:
                self.robotVelocities[index, 0] = self.robotMaxVelocity

            if robotPosMag > 0.0 and robotPosMag >= self.targetMaxVelocity:
                arcsinValue = self.targetMaxVelocity * numpy.sin(self.targetHeadings[index, 0] - relativeHeading) / robotPosMag
                self.robotHeadings[index, 0] = relativeHeading + numpy.arcsin(arcsinValue)
            else:
                self.robotHeadings[index, 0] = relativeHeading + numpy.arcsin(0.0)

            # Update the robot's X and Y velocities
            velocityX = self.robotVelocities[index, 0] * numpy.cos(self.robotHeadings[index, 0])
            velocityY = self.robotVelocities[index, 0] * numpy.sin(self.robotHeadings[index, 0])

            # Give the robot its new position
            self.robotPositions[index, 0] = self.robotPositions[index-1, 0] + velocityX * self.deltaTime
            self.robotPositions[index, 1] = self.robotPositions[index-1, 1] + velocityY * self.deltaTime

        # Print the maps based on this set of data
        self.printGraphs(targetPath)

    def addNoise(self, index):
        noise = self.noiseSTD * numpy.random.randn(1, 2)
        self.targetPositions[index, 0] += noise[0, 0]
        self.targetPositions[index, 1] += noise[0, 1]

    def printGraphs(self, path):
        # Create the legend patches
        targetPatch = patches.Patch(color='red', label="Target Path")
        robotPatch = patches.Patch(color='green', label="Robot Path")
        relativePatch = patches.Patch(color='blue', label="Relative Distance")

        # Plot the coordinates of the positions
        plot.plot(self.targetPositions[:, 0], self.targetPositions[:, 1], 'r.',
                  self.robotPositions[:, 0], self.robotPositions[:, 1], 'g.')

        plot.legend(handles=[targetPatch, robotPatch], loc='best')
        plot.savefig("Positions_" + path + ".png")
        plot.show()

        # Create a new set of graphs for the distance error between the robot and target
        plot.plot(self.relativePosAbsolute[:, 0], self.relativePosAbsolute[:, 1], 'b.')
        plot.legend(handles=[relativePatch], loc='best')
        plot.savefig("DisErr_" + path + ".png")
        plot.show()

        # Create the heading graph of robot, target, and relative headings
        times = []
        for index in range(0, self.iterations):
            times.append(index)

        targetPatch = patches.Patch(color='red', label="Target Heading")
        robotPatch = patches.Patch(color='green', label="Robot Heading")
        relativePatch = patches.Patch(color='blue', label="Relative Heading")

        plot.plot(times, self.targetHeadings[:, 0], 'r.',
                  times, self.robotHeadings[:, 0], 'g.',
                  times, self.relativeHeadings[:, 0], 'b.')
        plot.legend(handles=[targetPatch, robotPatch, relativePatch], loc='best')
        plot.savefig("Headings_" + path + ".png")
        plot.show()
