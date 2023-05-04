import math
from typing import Tuple

from wpilib import (
    Color8Bit,
    DigitalInput,
    DutyCycleEncoder,
    Mechanism2d,
    SmartDashboard,
)
from wpimath.controller import ArmFeedforward, PIDController
from rev import CANSparkMax

from robot.constants import *


class ArmPosition:
    """
    Preset position constants for the arm. They rely on the absolute encoders on the arm
    being attached in a specific orientation so that these vaules correspond to
    the same position it was when we got them.
    """

    HOME = (5.407518, 5.999611)
    SUBSTATION = (4.093430, 5.156882)
    HIGH = (2.976432, 3.154062)
    LOW = (3.587507, 4.466632)
    BACK = (5.407518, 2.951937)


class Arm:
    """
    The Arm subsystem
    """

    _lower_setpoint: float
    _upper_setpoint: float

    _lower_motor = CANSparkMax(LOWER_ARM_MOTOR, CANSparkMax.MotorType.kBrushless)

    _lower_ff = ArmFeedforward(**LOWER_ARM_FF)
    _lower_con = PIDController(**LOWER_ARM_PID)

    _upper_motor = CANSparkMax(UPPER_ARM_MOTOR, CANSparkMax.MotorType.kBrushless)

    _upper_ff = ArmFeedforward(**UPPER_ARM_FF)
    _upper_con = PIDController(**UPPER_ARM_PID)

    _lower_home = DigitalInput(LOWER_ARM_HOME)
    _upper_home = DigitalInput(UPPER_ARM_HOME)

    _lower_enc = DutyCycleEncoder(LOWER_ARM_ENCODER)
    _upper_enc = DutyCycleEncoder(UPPER_ARM_ENCODER)

    def __init__(self):
        self.set_position(ArmPosition.HOME)

        self._lower_motor.restoreFactoryDefaults()
        self._upper_motor.restoreFactoryDefaults()

        self._lower_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self._upper_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self._lower_motor.setInverted(True)
        self._upper_motor.setInverted(True)

        self._mech_2d = Mechanism2d(100, 60)

        self._mech_root = self._mech_2d.getRoot("ArmBase", 40, 15)
        self._mech_root.appendLigament("Pylon", 15, -90, 6, Color8Bit(0, 0, 255))
        self._mech_lower = self._mech_root.appendLigament("LowerArm", 20, 193)
        self._mech_upper = self._mech_lower.appendLigament("UpperArm", 16, 193)

        SmartDashboard.putData("ArmView", self._mech_2d)

    def set_position(self, pos: Tuple[float, float]) -> None:
        """
        Set the position, or setpoint of the arm, using a tuple: (lower, upper)
        """

        self._lower_setpoint = pos[0]
        self._upper_setpoint = pos[1]

    def bump_lower_position(self, bump: float):
        """
        'Bump' the lower arm position.
        Allows for manual adjustment of the set position.
        """
        self._lower_setpoint += bump

    def bump_upper_position(self, bump: float):
        """
        'Bump' the upper arm position.
        Allows for manual adjustment of the set position.
        """
        self._upper_setpoint += bump

    def get_lower_position(self) -> float:
        """
        Get the position of the lower arm in Radians
        """
        return self._lower_enc.getAbsolutePosition() * (2 * math.pi)

    def get_upper_position(self) -> float:
        """
        Get the position of the upper arm in Radians
        """
        return self._upper_enc.getAbsolutePosition() * (2 * math.pi)

    def is_lower_home(self) -> bool:
        """
        Get the value of the lower limit switch
        """
        return self._lower_home.get()

    def is_upper_home(self) -> bool:
        """
        Get the value of the upper limit switch
        """
        return self._upper_home.get()

    def periodic(self):
        lower_out = -self._lower_con.calculate(
            self.get_lower_position(), self._lower_setpoint
        )
        upper_out = -self._upper_con.calculate(
            self.get_upper_position(), self._upper_setpoint
        )

        lower_v = lower_out + self._lower_ff.calculate(self._lower_con.getSetpoint(), 0)

        upper_v = upper_out + self._upper_ff.calculate(self._upper_con.getSetpoint(), 0)

        if self.is_lower_home() and lower_out < 0.1:
            self._lower_motor.setVoltage(0)
        else:
            self._lower_motor.setVoltage(lower_v)

        if self.is_upper_home() and upper_out < 0.1:
            self._upper_motor.setVoltage(0)
        else:
            self._upper_motor.setVoltage(upper_v)

        self._mech_lower.setAngle(238 + math.degrees(self.get_lower_position()))
        self._mech_upper.setAngle(180 - math.degrees(self.get_upper_position()))

