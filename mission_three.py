################################################################################
# mission_three.py
#
# Description:
# [Describe What your mission does here]
#
# Author(s): [Your Name(s)]
# Date: [YYYY-MM-DD]
# Version: 1.0
#
# Dependencies:
# - robot
# - pybricks.tools
#
################################################################################

from robot import robot
from pybricks.tools import wait, StopWatch

def mission_three(r):
    print("Running Mission 3")
    # Your code goes here...
    # Sample Code: Test running the attachment motor until stalled
    r.ram.run_until_stalled(1000, duty_limit=30)

################################
# KEEP THIS AT THE END OF THE FILE
# This redirects to running main.
################################
if __name__ == "__main__":
    from main import main
    main()
