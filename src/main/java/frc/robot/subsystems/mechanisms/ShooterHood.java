// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ShooterHood extends SubsystemBase {
  /** Creates a new ShooterHood. */
  private final TalonFX m_shooterHood;
  private final TalonFX m_hoodRollers;
  private final CANcoder m_shooterHoodCANCoder;

  private final TalonFXConfiguration shooterHoodConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration hoodRollersConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration shooterHoodCANCoderConfig = new CANcoderConfiguration();

  private final Slot0Configs shooterHoodConfigPID = shooterHoodConfig.Slot0;
  private final Slot0Configs hoodRollersConfigPID = hoodRollersConfig.Slot0;

  private final double heightDifferenceToTarget = 1.11; // Meters
  public double ballVelocityToFlywheels = 6.07;
  // public double timeToShoot = 0.0;

  private final double shooterHoodGearRatio = 2.0 / 1.0;
  private final double shooterMotorToCANCoderRatio = 1.0 / 1.0;
  private final double startingPositionRotations = 0.233;

  private final double minimumAngle = 0;
  private final double maximumAngle = 74;

  private final DriveSubsystem m_robotDrive;
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);
  private final Turret m_turret;

  public ShooterHood(DriveSubsystem drive, Turret turret) {
    m_robotDrive = drive;
    m_turret = turret;

    shooterHoodCANCoderConfig.MagnetSensor.MagnetOffset = 0.8888888;
    m_shooterHoodCANCoder = new CANcoder(Constants.SwerveConstants.kHoodRotateCANCoderID);
    m_shooterHoodCANCoder.getConfigurator().apply(shooterHoodCANCoderConfig);

    shooterHoodConfig.Voltage.PeakForwardVoltage = 12;
    shooterHoodConfig.Voltage.PeakReverseVoltage = -12;
    shooterHoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    shooterHoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    shooterHoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    shooterHoodConfig.Feedback.FeedbackRemoteSensorID = Constants.SwerveConstants.kHoodRotateCANCoderID;
    // shooterHoodConfig.Feedback.FeedbackRotorOffset = 0.0;
    shooterHoodConfig.Feedback.SensorToMechanismRatio = shooterHoodGearRatio;
    shooterHoodConfig.Feedback.RotorToSensorRatio = shooterMotorToCANCoderRatio;
    shooterHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterHoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterHoodConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    shooterHoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterHoodConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    shooterHoodConfigPID.GravityType = GravityTypeValue.Arm_Cosine;
    // shooterHoodConfigPID.kG = -2.0; // Add 0.25 V output to overcome static friction
    shooterHoodConfigPID.kS = 8.0; // Add 0.25 V output to overcome static friction
    shooterHoodConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    shooterHoodConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    shooterHoodConfigPID.kP = 60.0; // A position error of 2.5 rotations results in 12 V output
    shooterHoodConfigPID.kI = 1.20; // no output for integrated error
    shooterHoodConfigPID.kD = 0.435; // A velocity error of 1 rps results in 0.1 V output
    shooterHoodConfig.withSlot0(shooterHoodConfigPID);

    hoodRollersConfig.Voltage.PeakForwardVoltage = 12;
    hoodRollersConfig.Voltage.PeakReverseVoltage = -12;
    hoodRollersConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    hoodRollersConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    // hoodRollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodRollersConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodRollersConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    hoodRollersConfigPID.kS = 0.05; // Add 0.25 V output to overcome static friction
    hoodRollersConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    hoodRollersConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    hoodRollersConfigPID.kP = 0.05; // A position error of 2.5 rotations results in 12 V output
    hoodRollersConfigPID.kI = 0.0; // no output for integrated error
    hoodRollersConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    hoodRollersConfig.withSlot0(hoodRollersConfigPID);

    m_shooterHood = new TalonFX(Constants.SwerveConstants.kHoodRotateMotorPort);
    m_hoodRollers = new TalonFX(Constants.SwerveConstants.kHoodRollerMotorPort);


    m_shooterHood.getConfigurator().apply(shooterHoodConfig);
    // m_shooterHood.setNeutralMode(NeutralModeValue.Brake);
    m_shooterHood.setPosition(startingPositionRotations);

    m_hoodRollers.getConfigurator().apply(hoodRollersConfig);
    m_hoodRollers.setNeutralMode(NeutralModeValue.Coast);
  }

  /**
   * @param angle Give the angle in degrees 
   */
  // Moves the shooter hood to the position given  
  public void setShooterHoodAngle(double angle) {
    if (angle < minimumAngle) {
      angle = minimumAngle;
    } else if (angle > maximumAngle) {
      angle = maximumAngle;
    }
    m_shooterHood.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }

  public void stopHoodAngle() {
    m_shooterHood.stopMotor();
    // m_shooterHood.setPosition(-9.4262695312);
  }

  // Returns the position of the shooter hood
  public double getPositionAngle() {
    return m_shooterHood.getPosition().getValueAsDouble();
  }

  public double getPositionRadians() {
    if (m_shooterHood.getPosition().getValue() == null) {
      return 60.0 / 180.0 * Math.PI;
    }
    return m_shooterHood.getPosition().getValueAsDouble() * 2 * Math.PI;
  }

  public void resetHood() {
    m_shooterHood.setPosition(startingPositionRotations);
  }

  public void setHoodRollerSpeed(double speed) {
    if (speed > 1.0) {speed = 1.0;}
    m_hoodRollers.set(speed);
  }

  /**
  *Calculates the angle of the shooter hood and the speed of the flywheels for the optimal shot based on lowest required velocity of the ball
  @param robotPose The pose of the robot when the shot lands
  @return Returns an array of doubles [theta, flywheelVelocity]
  */
  public double[] calculateShooterHoodAngleFlywheelSpeeds(double[] robotPose) {
    Translation3d targetPose;
    if (Constants.shuttleMode) {
      targetPose = Constants.targetShuttlePosition;
    } else {
      targetPose = Constants.targetHubPosition;
    }
    double distanceToTargetX = targetPose.getX() - robotPose[0];
    double distanceToTargetY = targetPose.getY() - robotPose[1];

    double distanceToTarget = Math.sqrt(distanceToTargetX * distanceToTargetX + distanceToTargetY * distanceToTargetY);

    SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);

    double theta = 0.0;
    double thetaOffset45 = 0.0;
    double ballInitialVelocity = 0.0;
    double flyWheelVelocity = 0.0;

    theta = Math.atan((heightDifferenceToTarget + Math.sqrt(heightDifferenceToTarget * heightDifferenceToTarget + distanceToTarget * distanceToTarget)) / distanceToTarget);
    thetaOffset45 = theta + (theta - Math.toRadians(45));

    // theta -= Math.toRadians(2.5);

    if (Constants.shuttleMode) {
      theta = MathUtil.clamp(theta, Math.toRadians(58), Math.toRadians(74));
    }

    if (Double.isNaN(theta)) {
      theta = Math.toDegrees(74.0);
    }

    if (distanceToTarget > 4) {
      theta = 69;
    } else {
      theta = 72;
    }

    if (Constants.shuttleMode) {
      ballInitialVelocity = (distanceToTarget / Math.cos(getPositionRadians())) * Math.sqrt(Math.abs(9.8 / (2 * (distanceToTarget * Math.tan(getPositionRadians()) - heightDifferenceToTarget))));
    } else {
      if (distanceToTarget > 4) {
        ballInitialVelocity = (distanceToTarget / Math.cos(getPositionRadians())) * Math.sqrt(Math.abs(9.8 / (2 * (distanceToTarget * Math.tan(getPositionRadians()) - heightDifferenceToTarget))));
      }
      ballInitialVelocity = (distanceToTarget / Math.cos(Math.toRadians(74.0))) * Math.sqrt(Math.abs(9.8 / (2 * (distanceToTarget * Math.tan(Math.toRadians(74.0)) - heightDifferenceToTarget))));
    }

    if (Double.isNaN(ballInitialVelocity)) {
      ballInitialVelocity = 10;
    }

    if (distanceToTarget < 3.0) {
      ballVelocityToFlywheels = 6.25;
    } else {
      ballVelocityToFlywheels = 6.07;
    }

    if (!Constants.shuttleMode) {
      ballVelocityToFlywheels = 6.16 - (distanceToTarget - 2.5) * 0.1;
    }

    flyWheelVelocity = ballInitialVelocity * ballVelocityToFlywheels;

    // Constants.timeToShoot = distanceToTarget / (ballInitialVelocity * Math.cos(getPositionRadians() + 90.0 * Math.PI / 180.0)) * 1.42; // 1.5833
    Constants.timeToShoot = distanceToTarget / (ballInitialVelocity * Math.cos(Math.toRadians(74.0))); // 1.42
    
    if (Constants.shuttleMode) {
      flyWheelVelocity *= 0.6;
    }

    Constants.hoodAngle = Math.toDegrees(theta);
    if (Constants.shuttleMode) {
      Constants.flywheelSpeed = MathUtil.clamp(flyWheelVelocity, 0,60);
    } else {
      if (distanceToTarget < 2) {
        Constants.flywheelSpeed = MathUtil.clamp(flyWheelVelocity / (distanceToTarget * .5),0,88);
      } else {
        Constants.flywheelSpeed = MathUtil.clamp(flyWheelVelocity * (distanceToTarget *.5),0,88);
      }
    }

    SmartDashboard.putNumber("TargetPoseX", targetPose.getX());
    SmartDashboard.putNumber("DistanceToTargetX", distanceToTargetX);
    SmartDashboard.putNumber("DistanceToTarget", distanceToTarget);
    SmartDashboard.putNumber("BallInitalVelocty", ballInitialVelocity);
    SmartDashboard.putNumber("ShooterHoodRadians", getPositionRadians());

    return new double[] {theta, flyWheelVelocity};
  }

  public void stopHoodRollers() {
    m_hoodRollers.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Hood Angle", getPositionAngle() * 360);
    SmartDashboard.putNumber("Shooter Hood Angle Error", m_shooterHood.getClosedLoopError().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Shooter Hood Rollers RPM", m_hoodRollers.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Shooter Hood Rollers Current", m_hoodRollers.getSupplyCurrent().getValueAsDouble());
 
    SmartDashboard.putNumber("Constants Hood Angle", Constants.hoodAngle);
    SmartDashboard.putNumber("Constants Flywheel Speed", Constants.flywheelSpeed);
    SmartDashboard.putNumber("Time To Shoot", Constants.timeToShoot);
    SmartDashboard.putString("Shuttle Preset", Constants.targetShuttleRelativePosition);
    calculateShooterHoodAngleFlywheelSpeeds(m_turret.getPoseWhenShotLands());
  }
}