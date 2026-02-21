// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.Indexer;
import frc.robot.subsystems.mechanisms.ShooterFlywheels;
import frc.robot.subsystems.mechanisms.ShooterHood;
import frc.robot.subsystems.mechanisms.Spindexer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final ShooterHood m_shooterHood;
  private final Spindexer m_spindexer;
  private final ShooterFlywheels m_shooterFlywheels;
  private final Indexer m_indexer;
  public ShootCommand(ShooterHood hd, Spindexer sp, ShooterFlywheels sf, Indexer idx) {
    m_shooterHood = hd;
    m_spindexer = sp;
    m_shooterFlywheels = sf;
    m_indexer = idx;
    addRequirements(m_shooterHood);
    addRequirements(m_spindexer);
    addRequirements(m_shooterFlywheels);
    addRequirements(m_indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_spindexer.setSpindexerSpeed(-0.8);
    m_indexer.setIndexerSpeed(0.8);
    m_shooterHood.setShooterHoodAngle(60);
    m_shooterHood.setHoodRollerSpeed(0.2);
    m_shooterFlywheels.setShooterFlywheelsSpeed(1.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.robotPositionXString == "OpponentTrench" || Constants.robotPositionXString == "AllianceTrench") {
      m_shooterHood.stopHoodAngle();
    } else {
      m_shooterHood.setShooterHoodAngle(60);
    }
    m_spindexer.setSpindexerSpeed(-0.8);
    m_indexer.setIndexerSpeed(0.8);
    m_shooterHood.setHoodRollerSpeed(0.2);
    m_shooterFlywheels.setShooterFlywheelsSpeed(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_spindexer.stopSpindexer();
      m_indexer.stopIndexer();
      m_shooterHood.setHoodRollerSpeed(0);
      m_shooterFlywheels.stopShooterFlywheels();
      m_shooterHood.stopHoodAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
