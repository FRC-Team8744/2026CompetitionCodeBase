// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
public class RemainShooting extends Command {
  /** Creates a new TeleopIntake. */
  private final Intake m_intake;
  private final IntakePivot m_intakePivot;
  private final Turret m_turret;
  private final Indexer m_indexer;
  private final ShooterFlywheels m_shooterFlywheels;
  private final ShooterHood m_shooterHood;
  private final Spindexer m_spindexer;
  private final ShooterHoodToZero m_shooterHoodToZero;
  // private final Turret m_turret;
  public RemainShooting(Intake in, IntakePivot inp, Turret tur, Indexer idx, ShooterFlywheels shf, ShooterHood shh, Spindexer sp, ShooterHoodToZero shtz) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = in;
    m_intakePivot = inp;
    m_turret = tur;
    m_indexer = idx;
    m_shooterFlywheels = shf;
    m_shooterHood = shh;
    m_spindexer = sp;
    m_shooterHoodToZero = shtz;

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
    // m_intake.setIntakeSpeed(0.9);
    // m_intakePivot.intakeDown(-2400); // -1500

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.shootWhileIntake || Constants.shuttleMode) {
      if (Constants.shuttleMode) {
        if (Constants.robotPositionXString == "AllianceTrench" || Constants.robotPositionXString == "OpponentTrench") {
          CommandScheduler.getInstance().schedule(m_shooterHoodToZero);
        } else {
          // m_shooterHood.setShooterHoodAngle(Constants.hoodAngle);
        }
      }
      m_shooterHood.setHoodRollerSpeed(Constants.flywheelSpeed / 50);
      m_turret.setTurretAngle(Constants.turretAngle);
      m_shooterFlywheels.setShooterFlywheelsRps(Constants.flywheelSpeed);
      if (Math.abs(m_shooterFlywheels.getLeftFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9) && Math.abs(m_shooterFlywheels.getRightFlywheelVelocity()) >= (Constants.flywheelSpeed * 60 * 0.9)) {
        if (Constants.shouldShoot) {
          m_indexer.setIndexerSpeed(1.0);
          m_spindexer.setSpindexerSpeed(-0.67);
        } else {
          m_indexer.stopIndexer();
          m_spindexer.stopSpindexer();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}