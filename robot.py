################################################################################
# robot.py
#
# Defines the Robot class for SPIKE Prime, handling motor and sensor initialization,
# drive base configuration, and additional utility functions. This module also 
# centralizes all necessary imports so that mission files can use robot features 
# without redundant imports.
#
# Author: Bolton Robotics
# Date: 2025-03-15
# Version: 1.0
#
# Dependencies:
# - pybricks.pupdevices (Motor, ColorSensor)
# - pybricks.parameters (Port, Direction, Axis, Side, Stop, Color, Button, Icon)
# - pybricks.robotics (DriveBase)
# - pybricks.hubs (PrimeHub)
# - pybricks.tools (wait, StopWatch)
#
# Configuration:
# - Defines port mappings for motors and sensors
# - Sets up robot dimensions, motor directions, and display orientation
# - Provides battery level indication and other helper functions
#
################################################################################
# Bring in all the libraries needed by all file here
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Axis, Side, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub
from pybricks.tools import wait
from pybricks.parameters import (
    Port,
    Direction,
    Axis,
    Side,
    Stop,
    Color,
    Button,
    Icon,
)
from typing import Final, Mapping, Protocol

################################################################################
#### <--------------------- BEGIN ROBOT CONFIGURATION --------------------> ####
################################################################################

#############################################
# ROBOT DIAGRAM
############################################# 
# The base robot is required to have: 
#   2 drive motors
# The base robot may optionally have:
#   0-2 attachment motors
#   0-2 color sensors
#
# These are labelled in the diagram below. Your robot doesn't need 
# to follow this exact layout.  The goal is more to identify the drive and
# attachment motors and indicate whether they are on the left side or the right
# side of your robot.
#  When present on your robot, the following definitions are used:
# "LCS"  Left Color Sensor
# "RCS"  Right Color Sensor
# "LAM"  Left Attachment Motor
# "RAM"  Right Attachment Motor
# "LDM"  Left Drive Motor
# "RDM"  Right Drive Motor
#
#                   FRONT
#        ___________________________
# LEFT  |                           |   RIGHT
# SIDE  |                           |   SIDE
#       |   [ LCS ]       [ RCS ]   |   <-- Left and Right Color Sensors
#       |                           |
#       |   [ LAM ]       [ RAM ]   |   <-- Left and Right Attachment Motors
#   -   |                           |   -
#  | |  |___________________________|  | |
#  | |  |                           |  | |
#  | |==|===[ LDM ]       [ RDM ]===|==| |   <-- Left and Right Drive Motors
#  | |  |___________________________|  | |
#  | |                                 | |
#   -               BACK                -
#
#    <---------AXLE_TRACK (mm)--------->
#
#
#############################################
# Define Robot Parameters Here
#############################################
# Configuration parameters used by DriveBase.  
# Methods which use these parameters include:
#   r.robot.turn()
#   r.robot.straight()
#   r.robot.arc()
#   r.robot.drive()
TIRE_DIAMETER: Final[int] = 57  # mm
AXLE_TRACK: Final[int] = 86  # distance between the wheels, mm
STRAIGHT_SPEED: Final[int] = 400  # mm/sec
STRAIGHT_ACCEL: Final[int] = 300  # mm/sec^2
TURN_RATE: Final[int] = 300  # deg/sec
TURN_ACCEL: Final[int] = 200  # deg/sec^2
#############################################
# Define Robot Port Mappings
#############################################
# This needs to be setup accoring to how you
# wired your robot.  If you don't have color sensor(s) or
# attachment motor(s) you can comment them out.
PORT_MAPPING = {
    "ldm": Port.C,  # Left Drive Motor (Required)
    "rdm": Port.D,  # Right Drive Motor (Required)
    "lam": Port.F,  # Left Attachment Motor (Optional)
    "ram": Port.E,  # Right Attachment Motor (Optional)
    #"lcs": Port.A,  # Left Color Sensor (Optional)
    #"rcs": Port.B,  # Right Color Sensor (Optional)
}
#############################################
# Define Brain Orientation
#############################################
# Indicate which side of the brain faces the front of the robot.
#  
#
#               FRONT
#            
#          ------<->------
#         |      USB      |
#         | A           B |
#         |               |
#   LEFT  | C           D |  RIGHT
#         |               |
#         | E           F |
#         |               |
#         |     <-()->    |             
#          ---------------
#               BOTTOM
DISPLAY_ORIENTATION: Final[Side] = Side.BOTTOM
#############################################
# Define Forward (Positive) Rotation For Each Motor
#############################################
# Define whether each motor's "forward" spinning direction
# is clockwise or counter-clockwise.  This is likely to be
# needed when drive or attatchment motors are installed inverted
# or upside down.  If your attachment motor is spinning in the wrong
# direction or if your robot spins in circles when you are trying to
# drive straight, you likely need to change one of these settings.abs
LDM_POSITIVE_DIRECTION: Final[Direction] = Direction.COUNTERCLOCKWISE
RDM_POSITIVE_DIRECTION: Final[Direction] = Direction.CLOCKWISE
LAM_POSITIVE_DIRECTION: Final[Direction] = Direction.CLOCKWISE
RAM_POSITIVE_DIRECTION: Final[Direction] = Direction.CLOCKWISE

################################################################################
#### <--------------------- END ROBOT CONFIGURATION --------------------> ####
################################################################################

# -----------------------------
# Minimal typing helpers for IntelliSense
# -----------------------------

class MotorLike(Protocol):
    def run_time(self, speed: int, time: int,
                 then: Stop = Stop.HOLD, wait: bool = True) -> object: ...
    def run_angle(self, speed: int, rotation_angle: int,
                  then: Stop = Stop.HOLD, wait: bool = True) -> object: ...
    def run_until_stalled(
        self,
        speed: int,
        then: Stop = Stop.COAST,
        duty_limit: int | None = None,
    ) -> object: ...
    def dc(self, duty: int) -> None: ...
    def stop(self) -> None: ...

class NoOpMotor:
    def run_time(self, speed: int, time: int,
                 then: Stop = Stop.HOLD, wait: bool = True) -> object:
        return None

    def run_angle(self, speed: int, rotation_angle: int,
                  then: Stop = Stop.HOLD, wait: bool = True) -> object:
        return None

    def run_until_stalled(
        self,
        speed: int,
        then: Stop = Stop.COAST,
        duty_limit: int | None = None,
    ) -> object:
        return 0

    def dc(self, duty: int) -> None:
        return None

    def stop(self) -> None:
        return None

class ColorSensorLike(Protocol):
    # Keep this minimal to match common line-following usage.
    def reflection(self) -> int: ...
    def ambient(self) -> int: ...

class NoOpColorSensor:
    """ColorSensor-shaped object that returns safe defaults when not installed."""

    def reflection(self) -> int:
        return 50

    def ambient(self) -> int:
        return 0

################################
# Define Robot Class
################################
# The Robot class describes your robot including which motors and sensors
# are present.  You may also choose to add custom methods like line following,
# a wheel cleaning routine, gyro calibration, etc. 
class robot:
    def __init__(self, port_mapping: Mapping[str, Port] = PORT_MAPPING) -> None:
        """
        SPIKE Prime robot wrapper.

        Design goal for kid-friendly mission editing:
        - No attribute should be Optional/None in common usage.
        - Missing attachments/sensors become NoOp objects instead of None,
        so VS Code doesn't show "attribute of None" squiggles.
        """

        """Initialize the robot with flexible port assignments."""
        self.port_mapping = port_mapping
        self.display_orientation = DISPLAY_ORIENTATION
        
        try:
            # Axis negation is valid in Pybricks but rejected by type stubs, so ignore typing
            # fmt: off
            self.hub = PrimeHub(top_side=Axis.Z, front_side=-Axis.Y)  # pyright: ignore
            # fmt: on
            self.hub.display.orientation(self.display_orientation)
            self.hub.speaker.volume(50)
        except:
            print("Hub initialization error")

        # Ensure drive motors are defined, else fail
        if "ldm" not in self.port_mapping or "rdm" not in self.port_mapping:
            raise ValueError("Left and Right Drive Motors must be defined!")
        
        try:
            self.ldm = Motor(
                self.port_mapping["ldm"], positive_direction=LDM_POSITIVE_DIRECTION)
        except:
            print("Left drive motor initialization error")
            raise
        
        try:
            self.rdm = Motor(
                self.port_mapping["rdm"], positive_direction=RDM_POSITIVE_DIRECTION)
        except:
            print("Right drive motor initialization error")
            raise
        
        try:
            self.robot = DriveBase(
                self.ldm, self.rdm, TIRE_DIAMETER, AXLE_TRACK)
            self.robot.use_gyro(True)
            self.robot.settings(STRAIGHT_SPEED, STRAIGHT_ACCEL,
                            TURN_RATE, TURN_ACCEL)
        except:
            print("Drive base initialization error")
    
        # Initialize attachment motors (real or NoOp)
        self.lam: MotorLike = NoOpMotor()
        if "lam" in self.port_mapping:
            try:
                self.lam = Motor(
                    self.port_mapping["lam"],
                    positive_direction=LAM_POSITIVE_DIRECTION,
                )
            except:
                print("Left attachment motor initialization error")
        
        self.ram: MotorLike = NoOpMotor()
        if "ram" in self.port_mapping:
            try:
                self.ram = Motor(
                    self.port_mapping["ram"],
                    positive_direction=RAM_POSITIVE_DIRECTION,
                )
            except:
                print("Right attachment motor initialization error")
        
        # Color sensors (real or NoOp)
        self.lcs: ColorSensorLike = NoOpColorSensor()
        if "lcs" in self.port_mapping:
            try:
                self.lcs = ColorSensor(self.port_mapping["lcs"])
            except:
                print("Left color sensor initialization error")
        
        self.rcs: ColorSensorLike = NoOpColorSensor()
        if "rcs" in self.port_mapping:
            try:
                self.rcs = ColorSensor(self.port_mapping["rcs"])
            except:
                print("Right color sensor initialization error")

#####################################
# Add Custom Methods Here
#####################################

    # Check Battery Level
    def show_battery_level(self):
        """Display battery level using LED ring and matrix."""
        self.hub.display.off()
        current_voltage = self.hub.battery.voltage()
        print(f"Battery Voltage: {current_voltage} mV")

        voltage_levels = [6000, 6400, 6800, 7200, 8300]
        led_colors = [Color.RED, Color.RED, Color.ORANGE, Color.GREEN, Color.BLUE]

        for i, threshold in enumerate(voltage_levels):
            if current_voltage < threshold:
                led_color = led_colors[i]
                break
        else:
            led_color = Color.BLUE

        self.hub.light.on(led_color)

        battery_icon = [[0, 0, 100, 0, 0], [0, 100, 100, 100, 0], [0, 100, 100, 100, 0], [0, 100, 100, 100, 0], [0, 100, 100, 100, 0]]
        off_icon = [[0]*5 for _ in range(5)]

        if led_color in [Color.ORANGE, Color.RED]:
            for _ in range(5):
                self.hub.display.icon(battery_icon) # pyright: ignore
                wait(500)
                self.hub.display.icon(off_icon) # pyright: ignore
                wait(500)

################################
# KEEP THIS AT THE END OF THE FILE
# This redirects to running main.
################################
if __name__ == "__main__":
    from main import main
    main()
