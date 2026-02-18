// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterFlywheels extends SubsystemBase {
  /** Creates a new ShooterFlywheels. */
  private final TalonFX m_shooterFlywheelsLeft;
  private final TalonFX m_shooterFlywheelsRight;
  private final TalonFXConfiguration shooterFlywheelsConfig = new TalonFXConfiguration();
  private final Slot0Configs shooterFlywheelsConfigPID = shooterFlywheelsConfig.Slot0;
  // private final VelocityVoltage goalVelocityLeft = new VelocityVoltage(0);
  // private final VelocityVoltage goalVelocityRight = new VelocityVoltage(0);

  public ShooterFlywheels() {
    shooterFlywheelsConfig.Voltage.PeakForwardVoltage = 12;
    shooterFlywheelsConfig.Voltage.PeakReverseVoltage = -12;
    shooterFlywheelsConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    shooterFlywheelsConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    shooterFlywheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterFlywheelsConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    shooterFlywheelsConfigPID.kS = 0.0005; // Add 0.25 V output to overcome static friction
    shooterFlywheelsConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    shooterFlywheelsConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    shooterFlywheelsConfigPID.kP = 0.0005; // A position error of 2.5 rotations results in 12 V output
    shooterFlywheelsConfigPID.kI = 0.0; // no output for integrated error
    shooterFlywheelsConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    shooterFlywheelsConfig.withSlot0(shooterFlywheelsConfigPID);

    m_shooterFlywheelsLeft = new TalonFX(Constants.SwerveConstants.kShooterFlywheelLeftMotorPort);
    m_shooterFlywheelsRight = new TalonFX(Constants.SwerveConstants.kShooterFlywheelRightMotorPort);

    m_shooterFlywheelsLeft.getConfigurator().apply(shooterFlywheelsConfig);
    m_shooterFlywheelsLeft.setNeutralMode(NeutralModeValue.Brake);
    m_shooterFlywheelsLeft.setPosition(0);

    m_shooterFlywheelsRight.getConfigurator().apply(shooterFlywheelsConfig);
    m_shooterFlywheelsRight.setNeutralMode(NeutralModeValue.Brake);
    m_shooterFlywheelsRight.setPosition(0);
  }

  public void stopShooterFlywheels() {
    m_shooterFlywheelsLeft.stopMotor();
    m_shooterFlywheelsRight.stopMotor();
  }
  /**
  * @param speed Set the speed of the shooter flywheels 0-1
  **/ 
  public void setShooterFlywheelsSpeed(double speed) {
    // m_shooterFlywheelsLeft.setControl(goalVelocityLeft.withEnableFOC(false).withSlot(0).withVelocity(4000));
    // m_shooterFlywheelsRight.setControl(goalVelocityRight.withEnableFOC(false).withSlot(0).withVelocity(-4000));
    m_shooterFlywheelsLeft.set(speed);
    m_shooterFlywheelsRight.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Flywheels Left Motor Speed", m_shooterFlywheelsLeft.getVelocity().getValueAsDouble() * 60);
    SmartDashboard.putNumber("Shooter Flywheels Right Motor Speed", m_shooterFlywheelsRight.getVelocity().getValueAsDouble() * 60);
  }
}