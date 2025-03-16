################################################################################
# mission_four.py
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

def mission_four(r):
    print("Running Mission 4")
    # Your code goes here...
    # Sample code: Test the speaker
    notes = [
    "E4/4_", "G4/4", "E4/4", "E4/4", "A4/4", "E4/4", "D4/4",
    "E4/4_", "B4/4", "E4/4", "E4/4", "C4/4", "B3/4", "G3/4",
    "E4/4", "B3/4", "E4/4", "E4/4", "D4/4", "D3/4", "B3/4", "F#3/4", "E3/2."
    ]
    r.hub.speaker.play_notes(notes,360)

################################
# KEEP THIS AT THE END OF THE FILE
# This redirects to running main.
################################
if __name__ == "__main__":
    from main import main
    main()