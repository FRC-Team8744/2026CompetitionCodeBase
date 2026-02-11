// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_turret;
  private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  private final Slot0Configs turretConfigPID = turretConfig.Slot0;
  // TODO: Add gear ratio for the turret
  private final double turretGearRatio = 0.0 / 1.0;
  // TODO: Add values for minimum and maximum angle of the turret and starting position
  private final double startingPositionRotations = 0;
  private final double minimumAngle = -180;
  private final double maximumAngle = 180;
  // TODO: Make turret go to a position
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);
  CANcoder turretCancoder;  // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/CANcoder/src/test/java/CANcoderTest.java

  public Turret() {
    turretConfig.Voltage.PeakForwardVoltage = 12;
    turretConfig.Voltage.PeakReverseVoltage = -12;
    turretConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    turretConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = Constants.SwerveConstants.kTurretCANCoderID;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    turretConfigPID.kS = 0.0005; // Add 0.25 V output to overcome static friction
    turretConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    turretConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    turretConfigPID.kP = 0.0005; // A position error of 2.5 rotations results in 12 V output
    turretConfigPID.kI = 0.0; // no output for integrated error
    turretConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    turretConfig.withSlot0(turretConfigPID);

    m_turret = new TalonFX(Constants.SwerveConstants.kTurretMotorPort);

    m_turret.getConfigurator().apply(turretConfig);
    m_turret.setNeutralMode(NeutralModeValue.Brake);
    m_turret.setPosition(startingPositionRotations);

    turretCancoder = new CANcoder(Constants.kTurretCANCoderID);
  }

  /**
   * @param angle Give the angle in degrees 
   */
  // Moves the intake to the position given  
  public void setTurretAngle(double angle) {
    angle = optimizeAngle(angle);
    m_turret.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }

  // Returns the position of the intake
  public double getPositionAngle() {
    return m_turret.getPosition().getValueAsDouble() * 360;
  }

  public void shuttle() {
    
  }

  public void stopTurret() {
    m_turret.stopMotor();
    // m_turret.setPosition(-9.4262695312);
  }

  private double optimizeAngle(double targetAngle) {
    if (targetAngle > maximumAngle) {
      targetAngle -= 360;
    } else if (targetAngle < minimumAngle) {
      targetAngle += 360;
    }
    return targetAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Motor Angle", getPositionAngle());
    SmartDashboard.putNumber("Turret Abs Angle", turretCancoder.getPosition().getValueAsDouble());
  }
}