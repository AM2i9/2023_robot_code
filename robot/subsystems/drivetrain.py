from ntcore import NetworkTable, NetworkTableInstance
from wpilib import ADXRS450_Gyro, AnalogGyro, SPI, Field2d, SmartDashboard, Timer
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.kinematics import DifferentialDriveWheelSpeeds, ChassisSpeeds
from ctre import (
    NeutralMode,
    FeedbackDevice,
    InvertType,
    WPI_TalonFX,
    StatorCurrentLimitConfiguration,
)

from robot.constants import *


class Drivetrain:
    _gyro = ADXRS450_Gyro(SPI.Port.kOnboardCS0)
    _level = AnalogGyro(0)

    _left_motor = WPI_TalonFX(DRIVE_LB_MOTOR)
    _left_follower = WPI_TalonFX(DRIVE_LF_MOTOR)

    _right_motor = WPI_TalonFX(DRIVE_RB_MOTOR)
    _right_follower = WPI_TalonFX(DRIVE_RF_MOTOR)

    _left_controller = PIDController(**DRIVE_PID)
    _right_controller = PIDController(**DRIVE_PID)

    _ff = SimpleMotorFeedforwardMeters(**DRIVE_FF)

    _pose_estimator: DifferentialDrivePoseEstimator
    _field = Field2d()
    _limelight: NetworkTable = NetworkTableInstance.getDefault().getTable("limelight")

    def __init__(self):
        self._left_motor.configFactoryDefault()
        self._left_follower.configFactoryDefault()
        self._right_motor.configFactoryDefault()
        self._right_follower.configFactoryDefault()

        self._left_motor.setNeutralMode(NeutralMode.Brake)
        self._left_follower.setNeutralMode(NeutralMode.Brake)
        self._right_motor.setNeutralMode(NeutralMode.Brake)
        self._right_follower.setNeutralMode(NeutralMode.Brake)

        self._left_follower.follow(self._left_motor)
        self._right_follower.follow(self._right_motor)

        self._left_follower.setSensorPhase(False)
        self._right_follower.setSensorPhase(False)

        self._left_motor.setInverted(False)
        self._right_motor.setInverted(True)

        self._left_follower.setInverted(InvertType.FollowMaster)
        self._right_follower.setInverted(InvertType.FollowMaster)

        self._left_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        self._right_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)

        cur_limit = StatorCurrentLimitConfiguration(True, 100, 100, 0)

        self._left_motor.configStatorCurrentLimit(cur_limit)
        self._right_motor.configStatorCurrentLimit(cur_limit)
        self._right_follower.configStatorCurrentLimit(cur_limit)
        self._left_follower.configStatorCurrentLimit(cur_limit)

        self._pose_estimator = DifferentialDrivePoseEstimator(
            DRIVE_KINEMATICS,
            self.get_gyro_rotation(),
            self.get_left_encoder_pos(),
            self.get_right_encoder_pos(),
            Pose2d(),
        )

        SmartDashboard.putData(self._field)

    def calibrate_gyro(self) -> None:
        self._gyro.calibrate()
        self._level.calibrate()

    def reset_encoders(self) -> None:
        self._left_motor.setSelectedSensorPosition(0)
        self._right_motor.setSelectedSensorPosition(0)

    def get_gyro_heading(self) -> float:
        return self._gyro.getAngle()

    def get_gyro_rotation(self) -> Rotation2d:
        return self._gyro.getRotation2d()

    def talon_to_meters(self, rotations: float):
        wheel_rot = rotations * DRIVE_GEARBOX
        pos_mtrs = wheel_rot * DRIVE_ENC_DPR
        return pos_mtrs

    def get_left_encoder_vel(self):
        return self.talon_to_meters(
            self._left_motor.getSelectedSensorVelocity() / DRIVE_ENC_CPR * 10
        )

    def get_right_encoder_vel(self):
        return self.talon_to_meters(
            self._right_motor.getSelectedSensorVelocity() / DRIVE_ENC_CPR * 10
        )

    def get_left_encoder_pos(self):
        return self.talon_to_meters(
            self._left_motor.getSelectedSensorPosition() / DRIVE_ENC_CPR
        )

    def get_right_encoder_pos(self):
        return self.talon_to_meters(
            self._right_motor.getSelectedSensorPosition() / DRIVE_ENC_CPR
        )

    def tank_drive_volts(self, left: float, right: float) -> None:
        self._left_motor.setVoltage(left)
        self._right_motor.setVoltage(right)

    def use_wheel_speeds(self, speeds: DifferentialDriveWheelSpeeds):
        self.set_wheel_speeds(speeds.left, speeds.right)

    def set_wheel_speeds(self, left: float, right: float) -> None:
        l_ff = self._ff.calculate(left)
        r_ff = self._ff.calculate(right)

        l_out = self._left_controller.calculate(self.get_left_encoder_vel(), left)
        r_out = self._right_controller.calculate(self.get_right_encoder_vel(), right)

        self.tank_drive_volts(l_out + l_ff, r_out + r_ff)

    def arcade_drive(self, forward: float, rotation: float) -> None:
        wheel_speeds = DRIVE_KINEMATICS.toWheelSpeeds(
            ChassisSpeeds(-forward, 0.0, -rotation)
        )
        self.use_wheel_speeds(wheel_speeds)

    def get_pose(self):
        return self._pose_estimator.getEstimatedPosition()

    def periodic(self):
        self._pose_estimator.update(
            self.get_gyro_rotation(),
            self.get_left_encoder_pos(),
            self.get_right_encoder_pos(),
        )

        if self._limelight.containsKey("botpose"):
            entry = self._limelight.getNumberArray("botpose", [])

            if entry:
                # Limelight centers it's poses around the center of the field while
                # the estimator does it around the bottom left corner, so we offset
                # it here.
                pose = Pose2d(
                    entry[0] + FIELD_LENGTH / 2,
                    entry[1] + FIELD_WIDTH / 2,
                    Rotation2d.fromDegrees(entry[5]),
                )

                self._pose_estimator.addVisionMeasurement(
                    pose, Timer.getFPGATimestamp() - (entry[6] / 1000)
                )

        self._field.setRobotPose(self._pose_estimator.getEstimatedPosition())
