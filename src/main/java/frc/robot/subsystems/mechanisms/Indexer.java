// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final TalonFX m_indexerMotor;
  private final TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
  private final Slot0Configs indexerConfigPID = indexerConfig.Slot0;
  // private final VelocityVoltage goalVelocity = new VelocityVoltage(0);

  public Indexer() {
    indexerConfig.Voltage.PeakForwardVoltage = 12;
    indexerConfig.Voltage.PeakReverseVoltage = -12;
    indexerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    indexerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    indexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    indexerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    indexerConfigPID.kS = 0.05; // Add 0.25 V output to overcome static friction
    indexerConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    indexerConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    indexerConfigPID.kP = 0.05; // A position error of 2.5 rotations results in 12 V output
    indexerConfigPID.kI = 0.0; // no output for integrated error
    indexerConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    indexerConfig.withSlot0(indexerConfigPID);

    m_indexerMotor = new TalonFX(Constants.SwerveConstants.kIndexerMotorPort);

    m_indexerMotor.getConfigurator().apply(indexerConfig);
    m_indexerMotor.setNeutralMode(NeutralModeValue.Brake);
    m_indexerMotor.setPosition(0);
  }

  public void stopIndexer() {
    m_indexerMotor.stopMotor();
  }

  public void setIndexerSpeed(double speed) {
    // m_indexerMotor.setControl(goalVelocity.withEnableFOC(false).withSlot(0).withVelocity(speed));
    m_indexerMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Indexer Motor Speed", m_indexerMotor.getVelocity().getValueAsDouble());
  }
}