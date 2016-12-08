################################################################################
## testMain.py                                                                ##
##                                                                            ##
## Testing data set for checking validity of a virtual target path planning   ##
## as detailed in targetTracking.py.                                          ##
##                                                                            ##
## Brittany McGarr                                                            ##
## CPE 470 Autonomous Mobile Robots                                           ##
## Fall 2016                                                                  ##
################################################################################

# Import packages
import targetTracking


def runSimulation():
    # Create the robot data
    robot = targetTracking.RobotData()

    # Start the bot at the origin and standing still
    robot.robotPositions.append([0, 0])
    robot.robotVelocities.append(0)
    robot.robotHeadings.append(0)

    # Create a virtual target set
    # First test will be a square set of waypoints
    targetWaypoints = [[100, 300], [400, 300], [400, 100], [100, 100]]

    # Run the simulation, taking 200 data points per waypoint at 0.05 deltaTime
    for waypoint in targetWaypoints:
        # Each waypoint
        for timestep in range(0, 200):
            robot.update(timeDiff=0.05, targetPos=waypoint)

        # Take time to find the waypoint and snap a picture before proceeding
        #
        # found = robot.capture.findTarget()
        # while found < 1:
        #     found = robot.capture.findTarget()
        #     # Move the bot around in a circle, taking pictures every point until the target is found

	# Quick target confirmation temp code
	found = robot.capture.findTarget("banana")
	count = 1

	while found < 1 and count < 20:
		found = robot.capture.findTarget("banana")
		count += 1

    # Print the graph of the results
#    robot.printGraphs(path="test")

runSimulation()
