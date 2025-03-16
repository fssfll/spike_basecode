################################################################################
# menu.py
#
# Handles mission selection via the SPIKE Prime hub menu.
# Allows users to choose and execute predefined missions based on display 
# orientation, ensuring intuitive navigation.
#
# Author: Bolton Robotics
# Date: 2025-03-15
# Version: 1.0
#
# Dependencies:
# - robot (for accessing robot instance and display orientation)
# - pybricks.parameters (Side)
# - pybricks.tools (hub_menu for user selection)
# - mission_x.py files (each containing a specific mission routine)
#
# Functionality:
# - Dynamically adjusts menu order based on the hub’s display orientation.
# - Calls the appropriate mission function based on user selection.
#
################################################################################

import robot
from pybricks.parameters import Side
from pybricks.tools import hub_menu

def menu(r):
    from mission_one import mission_one
    from mission_two import mission_two
    from mission_three import mission_three
    from mission_four import mission_four
    from mission_five import mission_five
    from mission_six import mission_six
    from mission_seven import mission_seven
    from mission_eight import mission_eight
    from mission_nine import mission_nine
    # Change the ordering based on the orientation so the arrow 
    # selections make sense.
    if r.display_orientation == Side.BOTTOM or r.display_orientation == Side.LEFT:
        selected = hub_menu("1", "9", "8", "7", "6", "5", "4", "3", "2")
    else:
        selected = hub_menu("1", "2", "3", "4", "5", "6", "7", "8", "9") 
    # Based on the selection, run a program.
    if selected == "1":
        mission_one(r)
    elif selected == "2":
        mission_two(r)
    elif selected == "3":
        mission_three(r)
    elif selected == "4":
        mission_four(r)
    elif selected == "5":
        mission_five(r)
    elif selected == "6":
        mission_six(r)
    elif selected == "7":
        mission_seven(r)
    elif selected == "8":
        mission_eight(r)
    elif selected == "9":
        mission_nine(r)

################################
# KEEP THIS AT THE END OF THE FILE
# This redirects to running main.
################################
if __name__ == "__main__":
    from main import main
    main()