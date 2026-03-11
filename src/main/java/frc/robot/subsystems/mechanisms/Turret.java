// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.subsystems.DriveSubsystem;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_turret;
  private final CANcoder m_turretCANCoder = new CANcoder(Constants.SwerveConstants.kTurretCANCoderID);
  private final TalonFXConfiguration turretConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration turretCANCoderConfig = new CANcoderConfiguration();
  private final Slot0Configs turretConfigPID = turretConfig.Slot0;
  private final Timer m_shootTimer = new Timer();
  private final Timer m_shuttleTimer = new Timer();
  // TODO: Add gear ratio for the turret
  private final double turretGearRatio = 0.0 / 1.0;
  private final double turretMotorToSensorGearRatio = 15 / 116;
  // TODO: Add values for minimum and maximum angle of the turret and starting position
  private final double startingPositionRotations = 0;
  private final double minimumAngle = -180;
  private final double maximumAngle = 180;
  private Pose2d targetPose;
  private final DriveSubsystem m_drive;
  // private PIDController m_turnCtrl = new PIDController(0.014, 0.015, 0.0013);
  private double heading;
  private double goalAngle;
  private double m_output;
  private Pose2d initialPoseShoot;
  private Pose2d initialPoseShuttle;
  // TODO: Make turret go to a position
  private final PositionVoltage goalPosition = new PositionVoltage(0);

  public Turret(DriveSubsystem drive) {
    initialPoseShoot = drive.getEstimatedPose();
    initialPoseShuttle = drive.getEstimatedPose();
    m_shootTimer.start();
    m_shuttleTimer.start();
    turretCANCoderConfig.MagnetSensor.MagnetOffset = 0.3716666667;
    m_turretCANCoder.getConfigurator().apply(turretCANCoderConfig);

    turretConfig.Voltage.PeakForwardVoltage = 12;
    turretConfig.Voltage.PeakReverseVoltage = -12;
    turretConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    turretConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = Constants.SwerveConstants.kTurretCANCoderID;
    turretConfig.Feedback.RotorToSensorRatio = turretMotorToSensorGearRatio;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turretConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    turretConfigPID.kS = 12.5; // Add 0.25 V output to overcome static friction
    turretConfigPID.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
    turretConfigPID.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
    turretConfigPID.kP = 50.0; // A position error of 2.5 rotations results in 12 V output
    turretConfigPID.kI = 0.0; // no output for integrated error
    turretConfigPID.kD = 1.5; // A velocity error of 1 rps results in 0.1 V output
    turretConfig.withSlot0(turretConfigPID);

    m_turret = new TalonFX(Constants.SwerveConstants.kTurretMotorPort);

    m_turret.getConfigurator().apply(turretConfig);
    m_turret.setNeutralMode(NeutralModeValue.Brake);
    m_turret.setPosition(startingPositionRotations);

    // this.setDefaultCommand(Commands.run(() -> setTurretAngle(calculateGoalAngle()), this));

    m_drive = drive;
  }

  /**
   * @param angle Give the angle in degrees 
   */
  // Moves the turret to the position given  
  public void setTurretAngle(double angle) {
    angle = optimizeAngle(angle);
    m_turret.setControl(goalPosition.withEnableFOC(false).withSlot(0).withPosition(angle / 360));
  }


  // Returns the position of the turret
  public double getPositionAngle() {
    return m_turret.getPosition().getValueAsDouble() * 360;
  }

  public void shuttle() {
    
  }

  public void stopTurret() {
    m_turret.stopMotor();
    // m_turret.setPosition(-9.4262695312);
  }

  private double optimizeAngle(double targetAngle) {
    if (targetAngle > maximumAngle) {
      targetAngle -= 360;
    } else if (targetAngle < minimumAngle) {
      targetAngle += 360;
    }
    return targetAngle;
  }

  public void calculateGoalAngleShoot() {
    var alliance = DriverStation.getAlliance();
    Pose2d newPose;

    if (alliance.get() == DriverStation.Alliance.Blue) {
      targetPose = new Pose2d(4.620, 4.035, new Rotation2d());
    }
    else {
      targetPose = new Pose2d(11.920, 4.035, new Rotation2d());
    }

    // double distanceToTargetX = m_drive.getEstimatedPose().getX() - targetPose.getX();
    // double distanceToTargetY = m_drive.getEstimatedPose().getY() - targetPose.getY();

    // if (alliance.get() == DriverStation.Alliance.Red) {
    //   goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 180);
    // }
    // else {
    //   goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)));
    // }
    // goalAngle -= m_drive.getEstimatedPose().getRotation().getDegrees();


    if (m_shootTimer.hasElapsed(0.04)) {
      newPose = m_drive.getEstimatedPose();

      double xVelocity = (newPose.getX() - initialPoseShoot.getX()) / m_shootTimer.get();
      double yVelocity = (newPose.getY() - initialPoseShoot.getY()) / m_shootTimer.get();

      double positionWhenShotLandsX = xVelocity * Constants.timeToShoot + initialPoseShoot.getX();
      double positionWhenShotLandsY = yVelocity * Constants.timeToShoot + initialPoseShoot.getY();

      double distanceToTargetWhenShotLandsX = positionWhenShotLandsX - targetPose.getX();
      double distanceToTargetWhenShotLandsY = positionWhenShotLandsY - targetPose.getY();

      if (alliance.get() == DriverStation.Alliance.Red) {
        goalAngle = (Math.toDegrees(Math.atan(distanceToTargetWhenShotLandsY / distanceToTargetWhenShotLandsX)) - 180);
      }
      else {
        goalAngle = (Math.toDegrees(Math.atan(distanceToTargetWhenShotLandsY / distanceToTargetWhenShotLandsX)));
      }

      goalAngle -= m_drive.getEstimatedPose().getRotation().getDegrees();

      m_shootTimer.reset();

      initialPoseShoot = m_drive.getEstimatedPose();
    } else {
      
    }

    Constants.turretAngle = goalAngle;
    // return goalAngle;
  }

    public void calculateGoalAngleShuttle() {
    var alliance = DriverStation.getAlliance();

    if (alliance.get() == DriverStation.Alliance.Blue) {
      targetPose = new Pose2d(4.620, 4.035, new Rotation2d());
    }
    else {
      targetPose = new Pose2d(11.920, 4.035, new Rotation2d());
    }

    double distanceToTargetX = m_drive.getEstimatedPose().getX() - targetPose.getX();
    double distanceToTargetY = m_drive.getEstimatedPose().getY() - targetPose.getY();

    if (alliance.get() == DriverStation.Alliance.Red) {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)));
    }
    else {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 180);
    }
    goalAngle -= m_drive.getEstimatedPose().getRotation().getDegrees();

    

    Constants.turretAngle = goalAngle;
    // return goalAngle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Motor Angle", getPositionAngle());
    SmartDashboard.putNumber("Turret Motor Voltage", m_turret.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Turret goal angle", Constants.turretAngle);
    if (Constants.shuttleMode) {
      calculateGoalAngleShuttle();
    } else {
      calculateGoalAngleShoot();
    }
    SmartDashboard.putNumber("Shoot Timer Time", m_shootTimer.get());
  }
}