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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {
  /** Creates a new ShooterHood. */
  private final TalonFX m_shooterHood;
  private final TalonFX m_hoodRollers;

  private final TalonFXConfiguration shooterHoodConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration hoodRollersConfig = new TalonFXConfiguration();

  private final Slot0Configs shooterHoodConfigPID = shooterHoodConfig.Slot0;
  private final Slot0Configs hoodRollersConfigPID = hoodRollersConfig.Slot0;

  private final double heightDifferenceToTarget = 1.6;
  public final double ballVelocityToFlywheels = 0.0;
  public double timeToShoot = 0.0;

  private final double shooterHoodGearRatio = 2.0 / 1.0;
  private final double startingPositionRotations = 0.233;

  private final double minimumAngle = 0;
  private final double maximumAngle = 90;
  // TODO: Make shooter hood go to a position
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);

  public ShooterHood() {
    shooterHoodConfig.Voltage.PeakForwardVoltage = 12;
    shooterHoodConfig.Voltage.PeakReverseVoltage = -12;
    shooterHoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    shooterHoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    shooterHoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    shooterHoodConfig.Feedback.FeedbackRemoteSensorID = Constants.SwerveConstants.kHoodRotateCANCoderID;
    // shooterHoodConfig.Feedback.FeedbackRotorOffset = 0.0;
    shooterHoodConfig.Feedback.SensorToMechanismRatio = shooterHoodGearRatio;
    shooterHoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterHoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterHoodConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    shooterHoodConfigPID.kS = 8.0; // Add 0.25 V output to overcome static friction
    shooterHoodConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    shooterHoodConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    shooterHoodConfigPID.kP = 100.0; // A position error of 2.5 rotations results in 12 V output
    shooterHoodConfigPID.kI = 0.2; // no output for integrated error
    shooterHoodConfigPID.kD = 3.0; // A velocity error of 1 rps results in 0.1 V output
    shooterHoodConfig.withSlot0(shooterHoodConfigPID);

    hoodRollersConfig.Voltage.PeakForwardVoltage = 12;
    hoodRollersConfig.Voltage.PeakReverseVoltage = -12;
    hoodRollersConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    hoodRollersConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    hoodRollersConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodRollersConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    hoodRollersConfigPID.kS = 0.05; // Add 0.25 V output to overcome static friction
    hoodRollersConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    hoodRollersConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    hoodRollersConfigPID.kP = 0.05; // A position error of 2.5 rotations results in 12 V output
    hoodRollersConfigPID.kI = 0.0; // no output for integrated error
    hoodRollersConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    hoodRollersConfig.withSlot0(hoodRollersConfigPID);

    // hoodRotateCANCoderConfig.MagnetSensor.MagnetOffset = 0.0;

    m_shooterHood = new TalonFX(Constants.SwerveConstants.kHoodRotateMotorPort);
    m_hoodRollers = new TalonFX(Constants.SwerveConstants.kHoodRollerMotorPort);
    // m_hoodRotateCANCoder = new CANcoder(Constants.SwerveConstants.kHoodRotateCANCoderID);

    m_shooterHood.getConfigurator().apply(shooterHoodConfig);
    // m_shooterHood.setNeutralMode(NeutralModeValue.Brake);
    m_shooterHood.setPosition(startingPositionRotations);

    m_hoodRollers.getConfigurator().apply(hoodRollersConfig);
    m_hoodRollers.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * @param angle Give the angle in degrees 
   */
  // Moves the shooter hood to the position given  
  public void setShooterHoodAngle(double angle) {
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

  public void shuttle() {
    
  }

  public void setHoodRollerSpeed(double speed) {
    m_hoodRollers.set(speed);
  }

  /**
  *Calculates the angle of the shooter hood and the speed of the flywheels for the optimal shot based on lowest required velocity of the ball
  @param distanceToTarget the hypotenuse of the robot's position from the target
  @return Returns an array of doubles [theta, flywheelVelocity]
  */
  public double[] calculateShooterHoodAngleFlywheelSpeeds(double distanceToTarget) {
    double theta = 0.0;
    double thetaOffset45 = 0.0;
    double ballInitialVelocity = 0.0;
    double flyWheelVelocity = 0.0;

    theta = Math.atan((heightDifferenceToTarget + Math.sqrt(heightDifferenceToTarget * heightDifferenceToTarget + distanceToTarget * distanceToTarget)) / distanceToTarget);
    thetaOffset45 = theta + (theta - Math.toRadians(45));

    ballInitialVelocity = (distanceToTarget / Math.cos(theta)) * Math.sqrt(9.8 / (2 * (distanceToTarget * Math.tan(theta) - heightDifferenceToTarget)));

    flyWheelVelocity = ballInitialVelocity * ballVelocityToFlywheels;

    timeToShoot = distanceToTarget / (ballInitialVelocity * Math.cos(theta));

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
  }
}