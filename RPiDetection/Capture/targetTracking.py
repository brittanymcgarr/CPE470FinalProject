################################################################################
## capture.py                                                                 ##
##                                                                            ##
## Routine for applying Artificial Potential Field to a live feed of data     ##
## collected by the bot in order to get a velocity and signal next motion     ##
## path planning for interception of a target.                                ##
##                                                                            ##
## Brittany McGarr                                                            ##
## CPE 470 Autonomous Mobile Robots                                           ##
## Fall 2016                                                                  ##
################################################################################

# Import libraries

import math
import numpy
import numpy.linalg
import matplotlib.pyplot as plot
import matplotlib.patches as patches

# Camera manager
# import capture


class RobotData:

    # Constructor and initialization of data
    def __init__(self):
        self.deltaTime = 0.05
        self.attractFactor = 8.5

        # Target Data
        self.targetMaxVelocity = 1.2
        self.targetPositions = []
        self.targetHeadings = []

        # Robot Data
        self.robotMaxVelocity = 50
        self.robotPositions = []
        self.robotVelocities = []
        self.robotHeadings = []

        # Initialize relative states between robot and target
        self.relativePositions = []
        self.relativePosAbsolute = []
        self.relativeVelocities = []
        self.relativeHeadings = []

        # Initialize the camera data
        # self.capture = capture.Capture()
        self.targetShape = "faces"

    def update(self, timeDiff=0.05, targetPos=None):
        # Make sure we're getting target data, otherwise, return from the function
        if targetPos is None:
            return

        # Check that we got the correct number of position parameters in the target data
        if len(targetPos) == 2:
            self.targetPositions.append(targetPos)
        else:
            return

        # Hold on to the recent target position data
        targetX = targetPos[0]
        targetY = targetPos[1]

        # Set the target heading
        self.targetHeadings.append(math.atan2(targetY, targetX))

        # Compute the relative position of the virtual target and robot
        relativePosition = [targetX - self.robotPositions[-1][0], targetY - self.robotPositions[-1][1]]
        self.relativePositions.append(relativePosition)

        relativePositionX = relativePosition[0]
        relativePositionY = relativePosition[1]

        # Store the absolute distance values for plotting
        self.relativePosAbsolute.append(numpy.max([math.fabs(relativePositionX), math.fabs(relativePositionY)]))

        # Compute the relative heading of the virtual target and robot
        self.relativeHeadings.append(math.atan2(relativePositionY, relativePositionX))

        relativeHeading = self.relativeHeadings[-1]

        # Control and record the velocity and heading of the robot
        targetPosMag = numpy.linalg.norm(self.targetPositions[-1])
        targetPosMagSqr = targetPosMag * targetPosMag
        robotPosMag = numpy.linalg.norm(self.robotPositions[-1])
        relativePosMag = numpy.linalg.norm(self.relativePositions[-1])
        relativePosMagSqr = relativePosMag * relativePosMag

        self.robotVelocities.append(math.sqrt((self.targetMaxVelocity * self.targetMaxVelocity) + 2 *
                                                   self.attractFactor * relativePosMag * self.targetMaxVelocity *
                                                   math.fabs(numpy.cos(self.targetHeadings[-1] - relativeHeading)) +
                                                   (self.attractFactor * self.attractFactor) * relativePosMagSqr))

        # Maintain a maximal velocity
        if self.robotVelocities[-1] > self.robotMaxVelocity:
            self.robotVelocities[-1] = self.robotMaxVelocity

        if robotPosMag > 0.0 and robotPosMag >= self.targetMaxVelocity:
            arcsinValue = self.targetMaxVelocity * numpy.sin(self.targetHeadings[-1] - relativeHeading) / robotPosMag
            self.robotHeadings.append(relativeHeading + numpy.arcsin(arcsinValue))
        else:
            self.robotHeadings.append(relativeHeading + numpy.arcsin(0.0))

        # Update the robot's X and Y velocities
        velocityX = self.robotVelocities[-1] * numpy.cos(self.robotHeadings[-1])
        velocityY = self.robotVelocities[-1] * numpy.sin(self.robotHeadings[-1])

        # Give the robot its new position
        robotPosition = []
        robotPosition.append(self.robotPositions[-1][0] + velocityX * timeDiff)
        robotPosition.append(self.robotPositions[-1][1] + velocityY * timeDiff)
        self.robotPositions.append(robotPosition)

    # The data can also be given direction for a timestep to manually override following
    # (Such as a circular path around a target *hinthint*)
    def move(self, timeDiff=0.05, newPos=None, velocity=10, heading=0.0):
        if newPos is not None:
            velocityX = velocity * numpy.cos(heading)
            velocityY = velocity * numpy.sin(heading)

            robotPosition = []
            robotPosition.append(self.robotPositions[-1][0] + velocityX * timeDiff)
            robotPosition.append(self.robotPositions[-1][1] + velocityY * timeDiff)

            self.robotPositions.append(robotPosition)
            self.robotVelocities.append(velocity)
            self.robotHeadings.append(heading)

    def printGraphs(self, path):
        # Create the legend patches
        targetPatch = patches.Patch(color='red', label="Target Path")
        robotPatch = patches.Patch(color='green', label="Robot Path")
        relativePatch = patches.Patch(color='blue', label="Relative Distance")

        # Plot the coordinates of the positions
        listX = [item[0] for item in self.targetPositions]
        listY = [item[1] for item in self.targetPositions]
        botX = [item[0] for item in self.robotPositions]
        botY = [item[1] for item in self.robotPositions]

        plot.plot(listX, listY, 'r.', botX, botY, 'g.')

        plot.legend(handles=[targetPatch, robotPatch], loc='best')
        plot.savefig("Positions_" + path + ".png")
        plot.show()
