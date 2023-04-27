from math import pi

from wpilib import Units
from wpimath.kinematics import DifferentialDriveKinematcis

# ----- DRIVETRAIN -----

DRIVE_LB_MOTOR = 10
DRIVE_LF_MOTOR = 11

DRIVE_RB_MOTOR = 12
DRIVE_LB_MOTOR = 13

DRIVE_ENC_CPR = 2048
DRIVE_WHEEL_DIAMETER = Units.inchesToMeters(6)
DRIVE_ENC_DPR = DRIVE_WHEEL_DIAMETER * pi
DRIVE_GEARBOX = (34/40) * (14/50)

DRIVE_TRACK_WIDTH = Units.inchesToMeters(21)
DRIVE_KINEMATCIS = DifferentialDriveKinematics(DRIVE_TRACK_WIDTH)

DRIVE_MAX_SPEED = 6
DRIVE_MAX_ROT_SPEED = 6

DRIVE_PID = {"Kp": 3.1285, "Ki": 0, "Kd": 0}
DRIVE_FF = {"kS": 0.50892, "kV": 0.28201, "kA": 1.1083}

# ----- ARM ------

LOWER_ARM_MOTOR = 20
UPPER_ARM_MOTOR = 21

LOWER_ARM_HOME = 0
UPPER_ARM_HOME = 1

LOWER_ARM_ENCODER = 7
UPPER_ARM_ENCODER = 8

UPPER_ARM_PID = {"Kp": 3, "Ki": 0, "Kd": 0}
LOWER_ARM_PID = {"Kp": 3, "Ki": 0, "Kd": 0}

UPPER_ARM_FF = {"kS": 0.66617, "kG": 0.085621, "kV": 1.944, "kA": 0.046416}

LOWER_ARM_FF = {"kS": 0.38834, "kG": 0.0942, "kV": 2.0427, "kA": 0.23556}
