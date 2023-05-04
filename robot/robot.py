import wpimath
from wpilib import TimedRobot, XboxController

from robot import auto
from robot.subsystems.arm import Arm, ArmPosition
from robot.subsystems.drivetrain import Drivetrain
from robot.subsystems.claw import Claw
from robot.constants import *


class Robot(TimedRobot):
    arm = Arm()
    claw = Claw()
    drivetrain = Drivetrain()

    driver = XboxController(0)
    aux = XboxController(1)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        forward = wpimath.applyDeadband(self.driver.getLeftY(), 0.2) * DRIVE_MAX_SPEED
        rotation = (
            wpimath.applyDeadband(self.driver.getRightX(), 0.2) * DRIVE_MAX_ROT_SPEED
        )
        
        self.drivetrain.arcade_drive(forward, rotation)

        if self.aux.getBButtonPressed():
            self.arm.set_position(ArmPosition.HOME)

        if self.aux.getYButtonPressed():
            self.arm.set_position(ArmPosition.SUBSTATION)

        if self.aux.getAButtonPressed():
            self.arm.set_position(ArmPosition.LOW)

        if self.aux.getXButtonPressed():
            self.arm.set_position(ArmPosition.HIGH)

        if self.aux.getLeftBumperPressed():
            self.claw.toggle()

        self.arm.periodic()
        self.drivetrain.periodic()

    def autonomousInit(self):
        auto.run_auto(auto.COMMAND, self)

    def autonomousPeriodic(self):
        self.arm.periodic()
