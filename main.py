################################################################
# main.py
#
# Entry point for the SPIKE Prime robot program. 
# This script initializes the robot, displays battery status, 
# and starts the program selection menu.
#
# Author: Bolton Robotics
# Date: 2025-03-15
# Version: 1.0
#
# Dependencies:
# - robot.py (Handles robot initialization and core functions)
# - menu.py (Provides a program selection menu)
#
################################################################

import robot
import menu

###########
# Startup
###########
def main():
    # Instantiate the Robot
    r = robot.robot()

    # Show battery level on first power up
    r.show_battery_level()

    # Program select menu
    menu.menu(r)

if __name__ == "__main__":
    main()