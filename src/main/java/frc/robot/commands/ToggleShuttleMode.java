// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.Indexer;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.ShooterFlywheels;
import frc.robot.subsystems.mechanisms.ShooterHood;
import frc.robot.subsystems.mechanisms.Spindexer;
import frc.robot.subsystems.mechanisms.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleShuttleMode extends Command {
  /** Creates a new ToggleShuttleMode. */
  private final Turret m_turret;
  private final ShooterFlywheels m_shooterFlywheels;
  private final ShooterHoodToZero m_shooterHoodToZero;
  private final Spindexer m_spindexer;
  private final Indexer m_indexer;
  private final Intake m_intake;
  
  public ToggleShuttleMode(Turret tur, ShooterFlywheels shf, ShooterHoodToZero shhz, Spindexer spi, Indexer ind, Intake in) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = tur;
    m_shooterFlywheels = shf;
    m_shooterHoodToZero = shhz;
    m_spindexer = spi;
    m_indexer = ind;
    m_intake = in;
    addRequirements(m_turret);
    addRequirements(m_shooterFlywheels);
    addRequirements(m_spindexer);
    addRequirements(m_indexer);
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (Constants.shuttleMode) {
      Constants.shuttleMode = false;
      // m_turret.setTurretAngle(180);
      m_intake.stopIntake();
      m_spindexer.stopSpindexer();
      m_indexer.stopIndexer();
      m_shooterFlywheels.setShooterFlywheelsSpeed(0.4);
      CommandScheduler.getInstance().schedule(m_shooterHoodToZero);
    } else {
      Constants.shuttleMode = true;
      Constants.shootWhileIntake = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
