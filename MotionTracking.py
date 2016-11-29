"""
File Name: Project2a.py
Author: Aarron Stewart
Class: CPE470
Term: Fall 2016

Final Project
"""

# import libraries
import math
import numpy as np
import matplotlib.pyplot as plt


class bot():
    def __init__(self):
        self.cord = []
        self.vel = []


class targ():
    def __init__(self):
        self.cord = []
        self.total = 0


def main():
    # Initialize variables
    deltaT = 0.5
    lmbda = 0.25  # used for lambda variable
    velMax = 50  # Max speed for the robot
    error = []
    length = 100

    #  initialize robots instances
    robot = bot()
    relativeBot = bot()

    #  initialize target instance
    target = targ()

    #  target initialization
    #  set target initial position
    target.cord.append( ( 300 , 200 ))

    ########################################
    #  robot Initialization

    #  initial position of the robot
    robot.cord.append( ( 300 , 0 , 0 ) )

    #  initial velocity of the robot
    robot.vel.append( 0.0 )

    ########################################
    #  determine the difference between the robot and the target
    relativeX = ( target.cord[ 0 ][ 0 ] - robot.cord[ 0 ][ 0 ])
    relativeY = ( target.cord[ 0 ][ 1 ] - robot.cord[ 0 ][ 1 ])

    #  save the relative position of the robot and the target
    relativeBot.cord.append( ( relativeX , relativeY , 0 ))

    phi = math.atan2( relativeBot.cord[ 0 ][ 1 ] , relativeBot.cord[ 0 ][ 0 ] )
    robot.cord[ 0 ] = ( 300 , 0 , phi )
    tempArray = np.array( [ relativeX , relativeY ] )
    error.append( np.linalg.norm( tempArray ) )

    ########################################

    for index in range( 1 , length ):
        # set the trajectory for the target

        #####################################

        #  without noise
        qt_x = 300
        qt_y = 200

        #  assign the position of the target
        target.cord.append( ( qt_x, qt_y ) )

        #####################################

        #  get phi variable
        phi = math.atan2( relativeBot.cord[ index - 1 ][ 1 ] , relativeBot.cord[ index - 1 ][ 0 ] )

        #  enter the code here
        if (error[ index - 1 ]) > 0.25:

            tempArray = np.array( [ relativeBot.cord[ index - 1 ][ 0 ] , relativeBot.cord[ index - 1 ][ 1 ] ] )

            #  get the magnitude of the robot and target relative position
            magRobTarg = np.linalg.norm( tempArray )

            #  removed subvariableone due to there being no target velocity
            subVariableOne = math.pow( lmbda , 2 ) * math.pow( magRobTarg , 2 )
            intVelDifRobot = math.sqrt( subVariableOne  )
            robot.vel.append( min( intVelDifRobot , velMax ) )

            #  robot dynamic controller
            xCordUpdate = robot.cord[ index - 1 ][ 0 ] + ( deltaT * robot.vel[ index ]) * math.cos( robot.cord[ index - 1 ][ 2 ] )
            yCordUpdate = robot.cord[ index - 1 ][ 1 ] + ( deltaT * robot.vel[ index ]) * math.sin( robot.cord[ index - 1 ][ 2 ] )
            thetaCordUpdate = phi

            robot.cord.append( ( xCordUpdate , yCordUpdate , thetaCordUpdate ) )

            #  get relative position of robot and target
            relativeX = target.cord[ index ][ 0 ] - robot.cord[ index ][ 0 ]
            relativeY = target.cord[ index ][ 1 ] - robot.cord[ index ][ 1 ]

            relativeBot.cord.append( ( relativeX , relativeY , 0 ) )

            tempArray = np.array( [ relativeX , relativeY ] )
            error.append( np.linalg.norm( tempArray ) )

            #  increment deltaT by 0.05
            deltaT += 0.05

        else:
            #  robot dynamic controller
            xCordUpdate = robot.cord[ index - 1 ][ 0 ]
            yCordUpdate = robot.cord[ index - 1 ][ 1 ]
            thetaCordUpdate = phi

            robot.cord.append( ( xCordUpdate , yCordUpdate , thetaCordUpdate ) )

            #  get relative position of robot and target
            relativeX = target.cord[ index ][ 0 ] - robot.cord[ index ][ 0 ]
            relativeY = target.cord[ index ][ 1 ] - robot.cord[ index ][ 1 ]

            relativeBot.cord.append( ( relativeX , relativeY , 0 ) )

            tempArray = np.array( [ relativeX , relativeY ] )
            error.append( np.linalg.norm( tempArray ) )

        #####################################

        #  display the robot and target location tracking
    for index in range( 0 , length ):
        plt.scatter( robot.cord[ index ][ 0 ] , robot.cord[ index ][ 1 ] , s = 2 , color = "red" )
        plt.scatter( target.cord[ index ][ 0 ] , target.cord[ index ][ 1 ] , s = 2 , color = "blue" )

    plt.show( )

    #  display relative location
    for index in range( 0 , length ):
        plt.scatter( index , error[ index ] , s = 2 , color = "black" )

    plt.show()

    #  display theta for robot and target
    for index in range( 0 , length ):
        plt.scatter( index , robot.cord[ index ][ 2 ] , s = 2 , color = "black" )

    plt.show( )


main( )
