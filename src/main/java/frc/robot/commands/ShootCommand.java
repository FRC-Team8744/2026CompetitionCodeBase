// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.mechanisms.Indexer;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ShooterFlywheels;
import frc.robot.subsystems.mechanisms.ShooterHood;
import frc.robot.subsystems.mechanisms.Spindexer;
import frc.robot.subsystems.mechanisms.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  private final ShooterHood m_shooterHood;
  private final Spindexer m_spindexer;
  private final ShooterFlywheels m_shooterFlywheels;
  private final Indexer m_indexer;
  private final Turret m_turret;
  private final ShooterHoodToZero m_shooterHoodToZero;
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private Timer m_timer;

  public ShootCommand(ShooterHood hd, Spindexer sp, ShooterFlywheels sf, Indexer idx, Turret turret, ShooterHoodToZero shtz, Intake in, IntakePivot inp) {
    m_shooterHood = hd;
    m_spindexer = sp;
    m_shooterFlywheels = sf;
    m_indexer = idx;
    m_turret = turret;
    m_shooterHoodToZero = shtz;
    m_intake = in;
    m_intakePivot = inp;
    addRequirements(m_shooterHood);
    addRequirements(m_spindexer);
    addRequirements(m_shooterFlywheels);
    addRequirements(m_indexer);
    addRequirements(m_turret);
    addRequirements(m_intake);
    addRequirements(m_intakePivot);
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_spindexer.setSpindexerSpeed(-0.8);
    // m_indexer.setIndexerSpeed(0.8);
    // m_shooterHood.setShooterHoodAngle(55);
    // m_shooterHood.setHoodRollerSpeed(0.2);
    // m_shooterFlywheels.setShooterFlywheelsSpeed(1.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.visionShoot) {
      m_shooterHood.setShooterHoodAngle(Constants.hoodAngle);
      m_turret.setTurretAngle(Constants.turretAngle);
      m_shooterHood.setHoodRollerSpeed(Constants.flywheelSpeed / 83.3333333333);
      m_shooterFlywheels.setShooterFlywheelsRps(Constants.flywheelSpeed);
      if (Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9) && Math.abs(m_shooterFlywheels.getRightFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9)) {
        if (Constants.shouldShoot) {
          m_indexer.setIndexerSpeed(0.7);
          m_spindexer.setSpindexerSpeed(-0.5);
        } else {
          m_indexer.stopIndexer();
          m_spindexer.stopSpindexer();
        }
      }
    } else {
      m_shooterHood.setShooterHoodAngle(Constants.presetHoodAngle);
      m_shooterHood.setHoodRollerSpeed(Constants.presetFlywheelSpeed / 83.3333333333);
      m_shooterFlywheels.setShooterFlywheelsRps(Constants.presetFlywheelSpeed);
      if (Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) >= (Constants.presetFlywheelSpeed * 60 * 0.8) && Math.abs(m_shooterFlywheels.getRightFlywheelVelocity()) >= (Constants.presetFlywheelSpeed * 60 * 0.8)) {
        m_indexer.setIndexerSpeed(0.5);
        m_spindexer.setSpindexerSpeed(-0.5);
      }
    }
    m_intake.setIntakeSpeed(0.4);
    if (m_timer.hasElapsed(0.4)) {
      m_intakePivot.intakeDown(-650);
      m_timer.restart();
    } else {
      m_intakePivot.intakeDown(-1100);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_spindexer.stopSpindexer();
      m_indexer.stopIndexer();
      m_shooterFlywheels.stopShooterFlywheels();
      m_turret.stopTurret();
      CommandScheduler.getInstance().schedule(m_shooterHoodToZero);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
