// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;

public final class Constants {
  public static final int kMaxSpeedPercentAuto = 100; //This effects Drive speed in telop DONT ASK ME WHY
  public static final int kMaxSpeedPercentTeleop = 65; // 65
  public static final int kMaxAccelerationPercent = 100;
  public static final double kDriverSpeedLimit = 1; // sets how much the max speed is modified by when you press down on the left stick basicly make go slower the default is 1 btw 

  public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  public static final Slot0Configs driveConfigPID = driveConfig.Slot0;

  // public static final PowerDistribution PDH = new PowerDistribution(30, PowerDistribution.ModuleType.kRev);

  // 17.55 is the distance of the field in meters
  // This gets the points of the triangles to calc if it can strafe 

  public static RotationEnum isAutoRotate = RotationEnum.NONE;
  public static boolean isAutoXSpeed = false;
  public static double autoXSpeed = 0;
  public static double autoYSpeed = 0;
  public static double autoRotateSpeed = 0;
  public static boolean isAutoYSpeed = false;
  public static String robotPositionXString = "Alliance";
  public static String robotPositionYString = "None";
  public static boolean shuttleMode = false;
  public static double hoodAngle = 83.25;

  //TODO: Add positions for shuttling presets
  //TODO: Add turret and hood positions for shuttling presets

  public Constants() {
    configureKrakens();
  }

  public static final class MechanismConstants {}

  public static final class SwerveConstants {
    public static final double kMaxSpeedMetersPerSecond = (5.94 * kMaxSpeedPercentAuto) / 100;
    public static final double kMaxSpeedTeleop = (10.0 * kMaxSpeedPercentTeleop) / 100;

    // The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
    // The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
    // We use NWU here because the rest of the library, and math in general, use NWU axes convention.
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#axis-conventions
    // TODO: Get all motor ids for the swerve modules and enure the orientation of the modules is correct
    public static final int kFrontLeftDriveMotorPort = 1; // 8
    public static final int kFrontRightDriveMotorPort = 4; // 3
    public static final int kRearLeftDriveMotorPort = 10; // 17
    public static final int kRearRightDriveMotorPort = 7; // 20

    public static final int kFrontLeftTurningMotorPort = 2; // 10
    public static final int kFrontRightTurningMotorPort = 5; // 5
    public static final int kRearLeftTurningMotorPort = 11; // 19
    public static final int kRearRightTurningMotorPort = 8; // 22

    public static final int kFrontLeftMagEncoderPort = 3; // 9
    public static final int kFrontRightMagEncoderPort = 6; // 4
    public static final int kRearLeftMagEncoderPort = 12; // 18
    public static final int kRearRightMagEncoderPort = 9; // 21

    // TODO: Get all motor ids for the other mechanisms
    public static final int kIntakePivotMotorPort = 14;
    public static final int kIntakeMotorPort = 15;
    public static final int kSpindexerMotorPort = 16;
    public static final int kIndexerMotorPort = 17;
    public static final int kShooterFlywheelLeftMotorPort = 18;
    public static final int kShooterFlywheelRightMotorPort = 19;
    public static final int kHoodRollerMotorPort = 20;
    public static final int kHoodRotateMotorPort = 21;
    public static final int kHoodRotateCANCoderID = 22;
    public static final int kTurretMotorPort = 23;
    public static final int kTurretCANCoderID = 24;
    public static int kClimberMotorPort;

    // Only disable the steering angle optimizer when measuring the CANcoder offsets!
    public static final boolean DISABLE_ANGLE_OPTIMIZER = false;

    // Note: Zeroing the CanCoder in Tuner X doesn't seem to affect the reported absolute position.
    // TODO: Get new offsets of swerve modules
    public static final double kFrontLeftMagEncoderOffsetDegrees = 1 - 0.598877; // 0.125244; // 3
    public static final double kFrontRightMagEncoderOffsetDegrees = 1 - 0.999023; // 0.846191; // 6
    public static final double kRearLeftMagEncoderOffsetDegrees = 1 - 0.228516; // 0.224121; // 12 
    public static final double kRearRightMagEncoderOffsetDegrees = 1 - 0.625; // 0.248779; // 9

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(20.750);

    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.750);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
          // This is the order all swerve module references need to be in!
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),  // Front Left Quadrant
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right Quadrant
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Rear Left Quadrant
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));  // Rear Right Quadrant

    // TODO: Recalibrate pigeon 2
    public static final int kIMU_ID = 13;

    public static int kSwerveFL_enum = 0;
    public static int kSwerveFR_enum = 1;
    public static int kSwerveRL_enum = 2;
    public static int kSwerveRR_enum = 3;
  }

  public static final class ConstantsOffboard {
    public static final int kMaximumSparkMaxRPM = 6000;
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 5.27 / 1.0; // 6.03:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = (287 / 11) / 1.0; // ~26.09:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 20;

    public static final boolean DRIVE_MOTOR_PROFILED_MODE = true;
    /** Angle motor PID values for speed/acceleration limited mode. */
    // Reference: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    public static final double DRIVE_KP_PROFILED = 0.01;
    public static final double DRIVE_KI_PROFILED = 0.0;
    public static final double DRIVE_KD_PROFILED = 0.0;
    public static final double DRIVE_KF_PROFILED = 0.23;
    public static final double DRIVE_MAX_VEL_PROFILED = kMaximumSparkMaxRPM;  // Maximum Velocity, RPM
    public static final double DRIVE_MAX_ACC_PROFILED = 20000;  // Maximum Acceleration, RPM^2
    public static final double DRIVE_MAX_ERR_PROFILED = 0.02;  // Error tolerance of PID controller, rotations

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.25;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.25;

    public static final double KRAKEN_V = 0.32;
    public static final double KRAKEN_P = 0.11;
    public static final double KRAKEN_I = 0.48;
    public static final double KRAKEN_D = 0.01;

    public static final boolean ANGLE_MOTOR_PROFILED_MODE = false;
    /** Angle motor PID values for speed/acceleration limited mode. */
    // Reference: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/src/main/java/frc/robot/Robot.java
    public static final double ANGLE_KP_PROFILED = 0.00075;
    public static final double ANGLE_KI_PROFILED = 0.0;
    public static final double ANGLE_KD_PROFILED = 0.0;
    public static final double ANGLE_KF_PROFILED = 0.0003;
    public static final double ANGLE_MAX_VEL_PROFILED = kMaximumSparkMaxRPM;  // Maximum Velocity, RPM
    public static final double ANGLE_MAX_ACC_PROFILED = 20000;  // Maximum Acceleration, RPM^2
    public static final double ANGLE_MAX_ERR_PROFILED = 0.02;  // Error tolerance of PID controller, rotations

    /** Angle motor PID values. */
    public static final double KRAKENROTATION_P = 40.0;
    public static final double KRAKENROTATION_I = 0.0;
    public static final double KRAKENROTATION_D = 0.0;
    public static final double KRAKENROTATION_V = 0.1;
    public static final PIDConstants ANGLE_PID = new PIDConstants(KRAKENROTATION_P, KRAKENROTATION_I, KRAKENROTATION_D);
    
    /** Swerve constraints. */
    public static final double MAX_SPEED_IN_PERCENT = 100.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 0.1 * MAX_SPEED_IN_PERCENT;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND * 4/3;
    public static final double MAX_ANGULAR_DEGREES_PER_SECOND = Math.toDegrees(MAX_ANGULAR_RADIANS_PER_SECOND);

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = true;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
    public static final double kRotationDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = (4.4 * kMaxSpeedPercentAuto) / 100;
    public static final double kMaxAccelerationMetersPerSecondSquared = (30 * kMaxAccelerationPercent) / 100;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public void configureKrakens() {
    // Driving Configs
    driveConfig.Voltage.PeakForwardVoltage = 12;
    driveConfig.Voltage.PeakReverseVoltage = -12;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    driveConfigPID.kV = Constants.ConstantsOffboard.KRAKEN_V;
    driveConfigPID.kP = Constants.ConstantsOffboard.KRAKEN_P;
    driveConfigPID.kI = Constants.ConstantsOffboard.KRAKEN_I;
    driveConfigPID.kD = Constants.ConstantsOffboard.KRAKEN_D;
    driveConfig.withSlot0(driveConfigPID);
  }
}