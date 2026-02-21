// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.


// package frc.robot.subsystems.vision;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.PoseEstimate;
// import frc.robot.subsystems.DriveSubsystem;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class LimeLight4 extends SubsystemBase {
//   // private PhotonCamera camera = new PhotonCamera("Camera 1.0");
//   public String limelightName = "limelight";

//   // private double[] result;
//   private Pose2d estimatedRobotPose;
//   private double time;
//   private PoseEstimate poseEstimate;
//   private int aprilTagCount;
//   private double pigeonYaw;

//   public LimeLight4() {}

//   @Override
//   public void periodic() {
//     // result = LimelightHelpers.getBotPose_wpiBlue(limelightName);
//     // estimatedRobotPose = LimelightHelpers.toPose2D(result);
//     pigeonYaw = DriveSubsystem.m_imu.getYaw().getValueAsDouble();

//     // LimelightHelpers.SetRobotOrientation(limelightName, pigeonYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    
//     poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
//     estimatedRobotPose = poseEstimate.pose;
//     time = poseEstimate.timestampSeconds;
//     aprilTagCount = poseEstimate.tagCount;

//     // time = result[1];

//   }

//   public Pose2d getEstimatedRobotPose() {
//     return estimatedRobotPose;
//   }

//   public double getAprilTageTime() {
//     return time;
//   }

//   public int getAprilTagAmount() {
//     return aprilTagCount;
//   }
// }