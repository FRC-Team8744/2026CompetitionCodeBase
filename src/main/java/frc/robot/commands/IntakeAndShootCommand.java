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
public class IntakeAndShootCommand extends Command {
  /** Creates a new TeleopIntake. */
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final Turret m_turret;
  private final Indexer m_indexer;
  private final ShooterFlywheels m_shooterFlywheels;
  private final ShooterHood m_shooterHood;
  private final Spindexer m_spindexer;
  private final ShooterHoodToZero m_shooterHoodToZero;
  private final RemainShooting m_remainShooting; 
  private final Timer m_timer; 
  private final Timer m_stallTimer;
  // private final Turret m_turret;
  public IntakeAndShootCommand(Intake in, IntakePivot inp, Turret tur, Indexer idx, ShooterFlywheels shf, ShooterHood shh, Spindexer sp, ShooterHoodToZero shtz, RemainShooting rem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = in;
    m_intakePivot = inp;
    m_turret = tur;
    m_indexer = idx;
    m_shooterFlywheels = shf;
    m_shooterHood = shh;
    m_spindexer = sp;
    m_shooterHoodToZero = shtz;
    m_remainShooting = rem;
    m_timer = new Timer();
    m_stallTimer = new Timer();

    addRequirements(m_turret);
    addRequirements(m_indexer);
    addRequirements(m_shooterFlywheels);
    addRequirements(m_shooterHood);
    addRequirements(m_spindexer);
    addRequirements(m_intake);
    addRequirements(m_intakePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeSpeed(0.9);
    m_intakePivot.intakeDown(-2400); // -1500
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.shootWhileIntake || Constants.shuttleMode) {
      // if (Constants.shuttleMode) {
        // m_shooterHood.setShooterHoodAngle(Constants.hoodAngle);
        if (Constants.robotPositionXString == "AllianceTrench" || Constants.robotPositionXString == "OpponentTrench") {
          m_shooterHood.setShooterHoodAngle(74);
        } else {
          m_shooterHood.setShooterHoodAngle(Constants.hoodAngle);
        }
      // }
      if (Constants.shuttleMode) {
        m_shooterHood.setHoodRollerSpeed(Constants.flywheelSpeed / -1250);
      } else {
        m_shooterHood.setHoodRollerSpeed(Constants.flywheelSpeed / -350);
      }
      m_turret.setTurretAngle(Constants.turretAngle);
      m_shooterFlywheels.setShooterFlywheelsRps(Constants.flywheelSpeed);
      if (Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9) && Math.abs(m_shooterFlywheels.getRightFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9) && Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) > (5 * 60 * 0.9)) {
        if (Constants.shouldShoot) {
          m_indexer.setIndexerSpeed(1.0);
          m_spindexer.setSpindexerSpeed(-0.9);
        } else {
          m_indexer.stopIndexer();
          m_spindexer.stopSpindexer();
        }
      }
    }
    if (Constants.enableAntiStall && !Constants.shootWhileIntake && !Constants.shuttleMode) {
      if (m_timer.hasElapsed(0.25)) {
        if (m_intake.isMotorStalling()) {
          if (!m_stallTimer.isRunning()) {
            m_stallTimer.start();
          }
          m_intake.setIntakeSpeed(-0.7);
        }
        if (m_stallTimer.hasElapsed(0.02)) {
          m_intake.setIntakeSpeed(0.9);
          m_stallTimer.stop();
          m_stallTimer.reset();
        }
      }
    }
    SmartDashboard.putNumber("Stall Timer time", m_stallTimer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!Constants.shootWhileIntake && !Constants.shuttleMode) {
      m_intake.stopIntake();
      m_intakePivot.stopIntakePivot(); // -1150
    }
    m_timer.stop();
    m_timer.reset();
    
    if (Constants.shootWhileIntake || Constants.shuttleMode) {
      CommandScheduler.getInstance().schedule(m_remainShooting);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}