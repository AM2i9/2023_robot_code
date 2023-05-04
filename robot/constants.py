from math import pi

from wpimath.kinematics import DifferentialDriveKinematics

# quick function convert inches to meters
in2m = lambda x: x * 0.0254

# ----- FIELD -----

FIELD_LENGTH = in2m(54)
FIELD_WIDTH = in2m(32)

# ----- CLAW -----

CLAW_CHANNEL = 6
CLAW_PHOTO = 2

# ----- DRIVETRAIN -----

DRIVE_LB_MOTOR = 10
DRIVE_LF_MOTOR = 11

DRIVE_RB_MOTOR = 12
DRIVE_RF_MOTOR = 13

DRIVE_ENC_CPR = 2048
DRIVE_WHEEL_DIAMETER = in2m(6)
DRIVE_ENC_DPR = DRIVE_WHEEL_DIAMETER * pi
DRIVE_GEARBOX = (34 / 40) * (14 / 50)

DRIVE_TRACK_WIDTH = in2m(21)
DRIVE_KINEMATICS = DifferentialDriveKinematics(DRIVE_TRACK_WIDTH)

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
