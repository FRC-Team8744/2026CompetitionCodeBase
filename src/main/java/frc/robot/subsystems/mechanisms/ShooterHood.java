// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

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
  private final TalonFX m_hood;
  private final TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
  private final Slot0Configs hoodConfigPID = hoodConfig.Slot0;
  private final double startingPositionRotations = 0;
  // TODO: Change values for minimum and maximum angle of the hood
  private final double minimumAngle = -4600;
  private final double maximumAngle = 0;
  private final PositionVoltage goalPosition = new PositionVoltage(startingPositionRotations);

  CANcoder hoodCancoder;
  /** Creates a new ShooterHood. */
  public ShooterHood() {
    hoodConfig.Voltage.PeakForwardVoltage = 12;
    hoodConfig.Voltage.PeakReverseVoltage = -12;
    hoodConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    hoodConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    hoodConfigPID.kS = 0.05; // Add 0.25 V output to overcome static friction
    hoodConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    hoodConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    hoodConfigPID.kP = 0.05; // A position error of 2.5 rotations results in 12 V output
    hoodConfigPID.kI = 0.0; // no output for integrated error
    hoodConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    hoodConfig.withSlot0(hoodConfigPID);

    m_hood = new TalonFX(Constants.khoodMotorPort);

    m_hood.getConfigurator().apply(hoodConfig);
    m_hood.setNeutralMode(NeutralModeValue.Brake);
    m_hood.setPosition(startingPositionRotations);

    hoodCancoder = new CANcoder(Constants.kHoodCancoderID);
  }

    /**
   * @param angle Give the angle in degrees 
   */
  // Moves the hood to the position given  
  public void setHoodAngle(double angle) {
    if (angle < minimumAngle) {angle = minimumAngle;}
    if (angle > maximumAngle) {angle = maximumAngle;}
    m_hood.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }

  // Returns the position of the intake
  public double getPositionAngle() {
    return m_hood.getPosition().getValueAsDouble() * 360;
  }

  public void stophood() {
    m_hood.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Motor Angle", m_hood.getPosition().getValueAsDouble() * 360);
    SmartDashboard.putNumber("Hood Abs Angle", hoodCancoder.getPosition().getValueAsDouble());
  }
}
