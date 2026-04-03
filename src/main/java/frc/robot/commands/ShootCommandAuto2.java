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
public class ShootCommandAuto2 extends Command {
  /** Creates a new ShootCommand. */
  private final ShooterHood m_shooterHood;
  private final Spindexer m_spindexer;
  private final ShooterFlywheels m_shooterFlywheels;
  private final Indexer m_indexer;
  private final Turret m_turret;
  private Timer m_timer;
  private Timer m_startTimer;
  private Timer m_stallTimer;
  private boolean intakeUp = false;

  public ShootCommandAuto2(ShooterHood hd, Spindexer sp, ShooterFlywheels sf, Indexer idx, Turret turret) {
    m_shooterHood = hd;
    m_spindexer = sp;
    m_shooterFlywheels = sf;
    m_indexer = idx;
    m_turret = turret;
    addRequirements(m_shooterHood);
    addRequirements(m_spindexer);
    addRequirements(m_shooterFlywheels);
    addRequirements(m_indexer);
    addRequirements(m_turret);
    m_timer = new Timer();
    m_stallTimer = new Timer();
    m_startTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_startTimer.start();
    m_stallTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.visionShoot) {
      // m_shooterHood.setShooterHoodAngle(Constants.hoodAngle);
      m_turret.setTurretAngle(Constants.turretAngle);
      m_shooterHood.setHoodRollerSpeed(Constants.flywheelSpeed / -350);
      m_shooterFlywheels.setShooterFlywheelsRps(Constants.flywheelSpeed);
      if (Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9) && Math.abs(m_shooterFlywheels.getRightFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9)) {
        if (Constants.shouldShoot) {
          m_indexer.setIndexerSpeed(1);
          m_spindexer.setSpindexerSpeed(-0.67);
        } else {
          m_indexer.stopIndexer();
          m_spindexer.stopSpindexer();
        }
      }
    } else {
      // m_shooterHood.setShooterHoodAngle(Constants.presetHoodAngle);
      m_shooterHood.setHoodRollerSpeed(Constants.presetFlywheelSpeed / -350);
      m_shooterFlywheels.setShooterFlywheelsRps(Constants.presetFlywheelSpeed);
      if (Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) >= (Constants.presetFlywheelSpeed * 60 * 0.8) && Math.abs(m_shooterFlywheels.getRightFlywheelVelocity()) >= (Constants.presetFlywheelSpeed * 60 * 0.8)) {
        m_indexer.setIndexerSpeed(1);
        m_spindexer.setSpindexerSpeed(-0.67);
      }
    }
    if (Constants.enableAntiStall) {
      if (m_startTimer.hasElapsed(0.25)) {
        if (m_spindexer.isMotorStalling()) {
          if (!m_stallTimer.isRunning()) {
            m_stallTimer.start();
          }
          m_spindexer.setSpindexerSpeed(0.67);
        }
        if (m_stallTimer.hasElapsed(0.05)) {
          m_spindexer.setSpindexerSpeed(-0.67);
          m_stallTimer.stop();
          m_stallTimer.reset();
        }
      }
    } else {
      m_spindexer.setSpindexerSpeed(-0.67);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_spindexer.stopSpindexer();
      m_indexer.stopIndexer();
      m_shooterFlywheels.stopShooterFlywheels();
      m_turret.stopTurret();
      m_shooterHood.stopHoodRollers();
      // m_shooterHood.setShooterHoodAngle(72.0); // 75
      // CommandScheduler.getInstance().schedule(m_shooterHoodToZero);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}