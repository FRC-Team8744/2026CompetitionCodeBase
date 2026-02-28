// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.alignment;

import java.util.Vector;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.DriveModifier;
import frc.robot.RotationEnum;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToHub extends DriveModifier{
  private Pose2d targetPose;
  private PIDController m_turnCtrl = new PIDController(0.014, 0.015, 0.0013);
  private double heading;
  private double goalAngle;
  private double m_output;

public AlignToHub() {
    super(false, false, true);
}

  // Called when the command is initially scheduled.
  public void initialize() {
    m_turnCtrl.enableContinuousInput(-180, 180);
    m_turnCtrl.setTolerance(2.00);
    m_turnCtrl.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  public void doExecute(DriveSubsystem drive) {
    var alliance = DriverStation.getAlliance();

    if (alliance.get() == DriverStation.Alliance.Blue) {
      targetPose = new Pose2d(4.620, 4.035, new Rotation2d());
    }
    else {
      targetPose = new Pose2d(11.920, 4.035, new Rotation2d());
    }

    double distanceToTargetX = drive.getEstimatedPose().getX() - targetPose.getX();
    double distanceToTargetY = drive.getEstimatedPose().getY() - targetPose.getY();

    if (alliance.get() == DriverStation.Alliance.Red) {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)) - 180);
    }
    else {
      goalAngle = (Math.toDegrees(Math.atan(distanceToTargetY / distanceToTargetX)));
    }

    if (goalAngle > 180) goalAngle -= 360;
    if (goalAngle < -180) goalAngle += 360;

    heading = drive.getEstimatedPose().getRotation().getDegrees();

    m_turnCtrl.reset();
    m_turnCtrl.setSetpoint(goalAngle);

    m_output = MathUtil.clamp(m_turnCtrl.calculate(heading), -1.0, 1.0);

    // m_output = MathUtil.clamp(heading, -1.0, 1.0);

    if (Math.abs(m_turnCtrl.getError()) <= m_turnCtrl.getErrorTolerance()) {
      Constants.autoRotateSpeed = 0;
    }
    else {
      Constants.autoRotateSpeed = m_output * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND;
    }
  }

  @Override
  public boolean shouldRun(DriveSubsystem drive) {
    return Constants.isAutoRotate == RotationEnum.ALIGNTOHUB;
  }
}