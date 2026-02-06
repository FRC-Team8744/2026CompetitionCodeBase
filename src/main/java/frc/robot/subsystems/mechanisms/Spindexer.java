// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  /** Creates a new Spindexer. */
  private final TalonFX m_spindexer;
  private final TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();
  private final Slot0Configs spindexerConfigPID = spindexerConfig.Slot0;
  private final VelocityVoltage goalVelocity = new VelocityVoltage(0);

  public Spindexer() {
    spindexerConfig.Voltage.PeakForwardVoltage = 12;
    spindexerConfig.Voltage.PeakReverseVoltage = -12;
    spindexerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    spindexerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    spindexerConfigPID.kS = 0.0005; // Add 0.25 V output to overcome static friction
    spindexerConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    spindexerConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    spindexerConfigPID.kP = 0.0005; // A position error of 2.5 rotations results in 12 V output
    spindexerConfigPID.kI = 0.0; // no output for integrated error
    spindexerConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    spindexerConfig.withSlot0(spindexerConfigPID);

    m_spindexer = new TalonFX(Constants.SwerveConstants.kSpindexerMotorPort);

    m_spindexer.getConfigurator().apply(spindexerConfig);
    m_spindexer.setNeutralMode(NeutralModeValue.Brake);
    m_spindexer.setPosition(0);
  }

  public void stopSpindexer() {
    m_spindexer.stopMotor();
  }

  public void setSpindexerSpeed(double speed) {
    m_spindexer.setControl(goalVelocity.withEnableFOC(false).withSlot(0).withVelocity(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Spindexer Motor Angle", m_spindexer.getPosition().getValueAsDouble() * 360);
  }
}