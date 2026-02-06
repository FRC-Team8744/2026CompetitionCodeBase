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

public class Climber extends SubsystemBase {
  /** Creates a new ShooterFlywheels. */
  private final TalonFX m_climberMotor;
  private final TalonFXConfiguration climberConfig = new TalonFXConfiguration();
  private final Slot0Configs climberConfigPID = climberConfig.Slot0;
  // private final VelocityVoltage goalVelocity = new VelocityVoltage(0);

  public Climber() {
    climberConfig.Voltage.PeakForwardVoltage = 12;
    climberConfig.Voltage.PeakReverseVoltage = -12;
    climberConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    climberConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    climberConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    climberConfigPID.kS = 0.0005; // Add 0.25 V output to overcome static friction
    climberConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    climberConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    climberConfigPID.kP = 0.0005; // A position error of 2.5 rotations results in 12 V output
    climberConfigPID.kI = 0.0; // no output for integrated error
    climberConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    climberConfig.withSlot0(climberConfigPID);

    m_climberMotor = new TalonFX(Constants.SwerveConstants.kClimberMotorPort);

    m_climberMotor.getConfigurator().apply(climberConfig);
    m_climberMotor.setNeutralMode(NeutralModeValue.Brake);
    m_climberMotor.setPosition(0);
  }

  public void stopClimber() {
    m_climberMotor.stopMotor();
  }

  public void setClimberSpeed(double speed) {
    // m_climberMotor.setControl(goalVelocity.withEnableFOC(false).withSlot(0).withVelocity(speed));
    m_climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Motor Speed", m_climberMotor.getVelocity().getValueAsDouble());
  }
}