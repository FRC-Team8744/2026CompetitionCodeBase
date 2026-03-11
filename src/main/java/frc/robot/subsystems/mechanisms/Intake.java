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

public class Intake extends SubsystemBase {
  /** Creates a new ShooterFlywheels. */
  private final TalonFX m_intakeMotor;
  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private final Slot0Configs intakeConfigPID = intakeConfig.Slot0;
  // private final VelocityVoltage goalVelocity = new VelocityVoltage(0);

  public Intake() {
    intakeConfig.Voltage.PeakForwardVoltage = 12;
    intakeConfig.Voltage.PeakReverseVoltage = -12;
    intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    intakeConfigPID.kS = 0.05; // Add 0.25 V output to overcome static friction
    intakeConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    intakeConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    intakeConfigPID.kP = 0.05; // A position error of 2.5 rotations results in 12 V output
    intakeConfigPID.kI = 0.0; // no output for integrated error
    intakeConfigPID.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
    intakeConfig.withSlot0(intakeConfigPID);

    m_intakeMotor = new TalonFX(Constants.SwerveConstants.kIntakeMotorPort);

    m_intakeMotor.getConfigurator().apply(intakeConfig);
    m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    m_intakeMotor.setPosition(0);
  }

  public void stopIntake() {
    m_intakeMotor.stopMotor();
  }

  public void setIntakeSpeed(double speed) {
    // m_intakeMotor.setControl(goalVelocity.withEnableFOC(false).withSlot(0).withVelocity(speed));
    m_intakeMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Speed", m_intakeMotor.getVelocity().getValueAsDouble());
  }
}