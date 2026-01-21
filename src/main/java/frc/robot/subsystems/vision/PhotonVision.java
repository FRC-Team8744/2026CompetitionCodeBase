// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.isInAreaEnum;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {
  private final Context context;

  private Result visionResult;
  
  public PhotonVision(Context context) {
    this.context = context;
    this.visionResult = Result.empty();
  }

  @Override
  public void periodic() {
    Result.Builder resultBuilder = Result.builder();

    PhotonPipelineResult result = context.camera.getLatestResult();
    resultBuilder.setApriltagTime(result.getTimestampSeconds());
    result.getTargets();

    if (result.hasTargets()) {
      PhotonTrackedTarget resultAprilTag = distillTarget(result);
      resultBuilder.setApriltag(resultAprilTag);

      // Start of check list
      Transform3d multiTagResult = result.getMultiTagResult().map(((m) -> m.estimatedPose.best)).orElse(null);

      var id = resultAprilTag.getFiducialId();
      Pose3d aprilTagPose3d = context.aprilTagFieldLayout.getTagPose(id).get();

      if (multiTagResult == null) {
        resultBuilder.setSingleTag(true);
        Transform3d cameraToTarget = resultAprilTag.getBestCameraToTarget();
        resultBuilder.setRobotPose(PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagPose3d, context.cameraToRobotOffset));
      } else {
        resultBuilder.setSingleTag(false);
        resultBuilder.setRobotPose(new Pose3d().plus(multiTagResult.plus(context.cameraToRobotOffset)));
      }
    }

    visionResult = resultBuilder.build();
  }

  private PhotonTrackedTarget distillTarget(PhotonPipelineResult result) {
    return result.getTargets().stream()
        .filter(((t) -> t.getFiducialId() == isInAreaEnum.areaEnum.getAprilTag()))
        .findAny()
        .orElseGet((() -> result.getBestTarget()));
  }

  // port: http://photonvision.local:5800

  public Result getVisionResult() {
    return visionResult;
  }

  public static class Context {
    private final PhotonCamera camera;
    private final Transform3d cameraToRobotOffset;
    private final AprilTagFieldLayout aprilTagFieldLayout;

    public Context(String cameraName, Transform3d cameraToRobotOffset, AprilTagFieldLayout aprilTagFieldLayout) {
      this.camera = new PhotonCamera(cameraName);
      this.cameraToRobotOffset = cameraToRobotOffset;
      this.aprilTagFieldLayout = aprilTagFieldLayout;
    }
  }

  public static class Result {
    public final Optional <PhotonTrackedTarget> apriltag;
    public final Optional <Pose2d> robotPose;
    public final double apriltagTime;
    public final boolean singleTag;

    private Result(PhotonTrackedTarget apriltag, Pose3d robotPose, double apriltagTime, boolean singleTag) {
      this.apriltag = Optional.ofNullable(apriltag);
      this.robotPose = Optional.ofNullable(robotPose).map((rp) -> rp.toPose2d());
      this.apriltagTime = apriltagTime;
      this.singleTag = singleTag;
    }

    public static Result empty() {
      return new Result(null, null, 0.0, true);
    }

    private static Builder builder() {
      return new Builder();
    }

    private static class Builder {
      private PhotonTrackedTarget apriltag = null;
      private Pose3d robotPose = null;
      private double apriltagTime = 0.0;
      private boolean singleTag = true;

      private Builder setApriltag(PhotonTrackedTarget apriltag) {
        this.apriltag = apriltag;
        return this;
      }

      private Builder setRobotPose(Pose3d robotPose) {
        this.robotPose = robotPose;
        return this;
      }

      private Builder setApriltagTime(double apriltagTime) {
        this.apriltagTime = apriltagTime;
        return this;
      }

      private Builder setSingleTag(boolean singleTag) {
        this.singleTag = singleTag;
        return this;
      }

      private Result build() {
        return new Result(apriltag, robotPose, apriltagTime, singleTag);
      }
    }
  }
}