// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.SwerveConstants;

public class SwerveModuleOffboard {
  // Drive motor
  private final TalonFX m_driveMotor;
  private final TalonFX m_driveEncoder;

  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final PositionVoltage turnPosition =  new PositionVoltage(0);

  private static final TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
  private static final Slot0Configs rotationConfigPID = rotationConfig.Slot0;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(0.32, 1.51, 0.27);

  // Turning motor
  private final TalonFX m_turningMotor;
  private final TalonFX m_turningEncoder;
  
  // Swerve module absolute encoder (wheel angle)
  private final CANcoder m_canCoder;
  private final double m_canCoderOffsetDegrees;

  SwerveModuleState state;
  private int DisplayCount = 0;

  /**
   * SwerveModuleOffboard - A SparkMax-based swerve module with canCoder wheel angle measurement
   *
   * @param driveMotorID The CAN ID of the drive motor.
   * @param turningMotorID The CAN ID of the turning motor.
   * @param magEncoderID The CAN ID of the magnetic encoder.
   * @param magEncoderOffsetDegrees The absolute offset of the magnetic encoder.
   */
  public SwerveModuleOffboard(int driveMotorID, int turningMotorID, int magEncoderID,
      double magEncoderOffsetDegrees) {
    // Create drive motor objects
    m_driveMotor = new TalonFX(driveMotorID);
    m_driveEncoder = m_driveMotor;

    // Create turning motor objects
    m_turningMotor = new TalonFX(turningMotorID);
    m_turningEncoder = m_turningMotor;

    // Create steering encoder objects (high resolution encoder)
    m_canCoder = new CANcoder(magEncoderID, "rio");
    m_canCoderOffsetDegrees = magEncoderOffsetDegrees;

    configureDevices();

    rotationConfig.Voltage.PeakForwardVoltage = 12;
    rotationConfig.Voltage.PeakReverseVoltage = -12;
    rotationConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    rotationConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    rotationConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    rotationConfig.Feedback.FeedbackRemoteSensorID = magEncoderID;
    rotationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    rotationConfig.ClosedLoopGeneral.ContinuousWrap = true;
    rotationConfigPID.kV = Constants.ConstantsOffboard.KRAKENROTATION_V;
    rotationConfigPID.kP = Constants.ConstantsOffboard.KRAKENROTATION_P;
    rotationConfigPID.kI = Constants.ConstantsOffboard.KRAKENROTATION_I;
    rotationConfigPID.kD = Constants.ConstantsOffboard.KRAKENROTATION_D;
    rotationConfig.withSlot0(rotationConfigPID);

    m_driveMotor.getConfigurator().apply(Constants.driveConfig);
    m_driveMotor.getConfigurator().setPosition(0);
    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    // }

    m_turningMotor.getConfigurator().apply(rotationConfig);
    // m_turningMotor.getConfigurator().setPosition(0);
    m_turningMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    state = desiredState;
    if (SwerveConstants.DISABLE_ANGLE_OPTIMIZER) {
      // Only enable when correcting wheel offsets!
      if (DisplayCount++ > 40) {
        System.out.println("CAUTION: Steering optimizer is disabled!");
        DisplayCount = 0;
      }
    } else {
      // Optimize the reference state to avoid spinning further than 90 degrees
      state.optimize(Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble()));
    }

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble())).getCos();

    // Set the PID reference states
    driveVelocity.Velocity = (state.speedMetersPerSecond * 60) / Constants.ConstantsOffboard.WHEEL_CIRCUMFERENCE;
    driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
    m_driveMotor.setControl(driveVelocity);
    turnPosition.Position = (state.angle.getRotations());
    m_turningMotor.setControl(turnPosition.withEnableFOC(false).withPosition(turnPosition.Position));
    // m_turningPID.setReference(state.angle.getRadians(), (ConstantsOffboard.ANGLE_MOTOR_PROFILED_MODE) ? SparkMax.ControlType.kMAXMotionPositionControl : SparkMax.ControlType.kPosition);
    
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoder() {
    m_driveEncoder.setPosition(0.0);
    m_turningEncoder.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    double velocity = m_driveMotor.getVelocity().getValueAsDouble();
    Rotation2d rot = Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble());
    return new SwerveModuleState(velocity, rot);
  }

  // public static double wrapTo360(double angleDeg) {
  //   angleDeg %= 360.0;
  //   if (angleDeg < 0) {
  //       angleDeg += 360.0;
  //   }
  //   return angleDeg;
  // }

  // public void optimize(double goalAngle, double currentAngle) {
  //   double x = goalAngle - currentAngle;
  //   double y = (goalAngle + 360) - currentAngle;
  //   double z = goalAngle - (currentAngle + 360);
  //   if (Math.abs(x) > 90.0 && Math.abs(y) > 90.0 && Math.abs(z) > 90.0) {
  //     state.speedMetersPerSecond *= -1;
  //     state.angle = Rotation2d.fromDegrees(goalAngle).rotateBy(Rotation2d.kPi);
  //     // Constants.yCheck = true;
  //   } 
  //   else if (Math.abs(x) > 90.0 && Math.abs(y) <= 90.0) {
  //     state.angle = Rotation2d.fromDegrees((goalAngle + 360));
  //     // Constants.yCheck = true;
  //   }
  //   else if (Math.abs(x) > 90.0 && Math.abs(z) <= 90.0) {
  //     state.angle = Rotation2d.fromDegrees((goalAngle - 360));
  //     // Constants.yCheck = true;
  //   }
  // }

  /**
   * Returns the CANcoder's measured turn angle in degrees.
   */
  public double getCanCoder() {
    var posVal = m_canCoder.getPosition();
    if(posVal.getStatus().isOK()) {
        double val = posVal.getValueAsDouble();
        return val * 360.0;
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
        return 0.0;
    }
  }

  /**
   * Returns the SparkMax internal encoder's measured turn angle in degrees.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(m_canCoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getDriveMotorPosition() {
    return m_driveMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the SparkMax internal encoder's measured position in meters.
   */
  public SwerveModulePosition getPosition() {
    double distance = m_driveMotor.getPosition().getValueAsDouble() * ConstantsOffboard.DRIVE_ROTATIONS_TO_METERS;
    // Rotation2d rot = new Rotation2d(m_driveEncoder.getPosition().getValueAsDouble() * ConstantsOffboard.ANGLE_GEAR_RATIO);
    Rotation2d rot = Rotation2d.fromRotations(m_canCoder.getAbsolutePosition().getValueAsDouble());
    return new SwerveModulePosition(distance, rot);
  }

  public double getVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  public double getDesiredPosition() {
    return turnPosition.Position;
  }
  
  public double getCurrent() {
    // return m_driveMotor.getOutput();
    return m_driveMotor.getTorqueCurrent().getValueAsDouble();
  }

   public double getTurnCurrent() {
    return m_turningMotor.getTorqueCurrent().getValueAsDouble();
  }

  public double getCurrentDriveAngle() {
    return (m_canCoder.getAbsolutePosition().getValueAsDouble());
  }

  private void configureDevices() {
    /* Configure CANcoder */
    var toApply = new CANcoderConfiguration();
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    toApply.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    toApply.MagnetSensor.MagnetOffset = m_canCoderOffsetDegrees;
    m_canCoder.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    m_canCoder.getPosition().setUpdateFrequency(100);
    m_canCoder.getVelocity().setUpdateFrequency(100);

    // According to this:
    // https://www.chiefdelphi.com/t/ctre-phoenix-pro-to-phoenix-6-looking-back-and-looking-ahead/437313/27
    // When using Phoenix6, the CanCoder should not have problems on startup as long as we wait for an update and check for errors.
    var posVal = m_canCoder.getAbsolutePosition().waitForUpdate(0.1); // This actaully waits that long! Don't call after init!
    if(posVal.getStatus().isOK()) {
        /* Perform seeding */
        // double val = posVal.getValueAsDouble();
        // m_turningEncoder.setPosition(Units.degreesToRadians(val * 360.0 - m_canCoderOffsetDegrees));
        // m_turningEncoder.setPosition(Units.degreesToRadians(val * 360.0));
    } else {
        /* Report error and retry later */
        System.out.println("Error reading CANcoder position! Robot will not drive straight!");
    }

    // Math check! Is reading the SparkMax position good enough?
    // CANcoder resolution in degrees: 360/4096 = 0.088 degrees
    // SparkMax encoder resolution: 360 / ( 48 positions * ANGLE_GEAR_RATIO (150/7) ) = 0.35 degrees
    // So is the CANcoder slightly better? Yes, but the CANcoder hardware spec:
    // https://store.ctr-electronics.com/content/user-manual/CANCoder%20User's%20Guide.pdf
    // says that the absolute position can be off by 1.44 degrees if there is rotation!
    // It may be possible to servo to m_canCoder.getPosition(), but then then CAN bus utilization will go way up.
    // Future project: Servo on the CANcoder position and see if CAN bus utilization is a problem.
  }
}