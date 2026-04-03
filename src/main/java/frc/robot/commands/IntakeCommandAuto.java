// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommandAuto extends Command {
  /** Creates a new TeleopIntake. */
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final Timer m_timer;
  // private final Turret m_turret;
  public IntakeCommandAuto(Intake in, IntakePivot inp/* , Turret tur*/) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = in;
    addRequirements(m_intake);
    m_intakePivot = inp;
    addRequirements(m_intakePivot);
    // m_turret = tur;
    // addRequirements(m_turret);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.autoIntake = true;
    m_intake.setIntakeSpeed(1);
    m_intakePivot.intakeDown(-600); // -2400
    m_timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.5)) {
      m_intakePivot.intakeDown(-2400);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_intakePivot.intakeDown(-2110);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !Constants.autoIntake;
  }
}