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

  private Result[] visionResult;
  
  public PhotonVision(Context context) {
    this.context = context;
    this.visionResult = Result.empty();
  }

  @Override
  public void periodic() {
    Result[] resultBuilder = new Result[context.numberOfCameras];

    for (int i = 0; i < context.numberOfCameras; i++) {
      CameraWithOffsets cameraWithOffsets = context.cameras[i];
      PhotonPipelineResult result = cameraWithOffsets.camera.getLatestResult();
      resultBuilder[i] = new Result(null, null, 0.0, true);
      resultBuilder[i].apriltagTime = result.getTimestampSeconds();
      result.getTargets();
  
      if (result.hasTargets()) {
        PhotonTrackedTarget resultAprilTag = distillTarget(result);
        resultBuilder[i].apriltag = Optional.ofNullable(resultAprilTag);

        // Start of check list
        Transform3d multiTagResult = result.getMultiTagResult().map(((m) -> m.estimatedPose.best)).orElse(null);

        var id = resultAprilTag.getFiducialId();
        Pose3d aprilTagPose3d = context.aprilTagFieldLayout.getTagPose(id).get();

        if (multiTagResult == null) {
          resultBuilder[i].singleTag = true;
          Transform3d cameraToTarget = resultAprilTag.getBestCameraToTarget();
          resultBuilder[i].robotPose = Optional.of(PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, aprilTagPose3d, cameraWithOffsets.cameraToRobotOffset).toPose2d());
        } else {
          resultBuilder[i].singleTag = false;
          resultBuilder[i].robotPose = Optional.of(new Pose3d().plus(multiTagResult.plus(cameraWithOffsets.cameraToRobotOffset)).toPose2d());
        }
      }
    }

    visionResult = resultBuilder;
  }

  private PhotonTrackedTarget distillTarget(PhotonPipelineResult result) {
    return result.getTargets().stream()
        .filter(((t) -> t.getFiducialId() == isInAreaEnum.areaEnum.getAprilTag()))
        .findAny()
        .orElseGet((() -> result.getBestTarget()));
  }

  // port: http://photonvision.local:5800

  public Result[] getVisionResult() {
    return visionResult;
  }

  public static class CameraWithOffsets {
    public final PhotonCamera camera;
    public final Transform3d cameraToRobotOffset;

    public CameraWithOffsets(String cameraName, Transform3d cameraToRobotOffset) {
      this.camera = new PhotonCamera(cameraName);
      this.cameraToRobotOffset = cameraToRobotOffset;
    }
  }

  public static class Context {
    private final CameraWithOffsets[] cameras;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final int numberOfCameras;

    public Context(AprilTagFieldLayout aprilTagFieldLayout, CameraWithOffsets...camera) {
      this.cameras = camera;
      this.aprilTagFieldLayout = aprilTagFieldLayout;
      this.numberOfCameras = camera.length;
    }
  }

  public static class Result {
    public Optional <PhotonTrackedTarget> apriltag;
    public Optional <Pose2d> robotPose;
    public double apriltagTime;
    public boolean singleTag;

    private Result(PhotonTrackedTarget apriltag, Pose3d robotPose, double apriltagTime, boolean singleTag) {
      this.apriltag = Optional.ofNullable(apriltag);
      this.robotPose = Optional.ofNullable(robotPose).map((rp) -> rp.toPose2d());
      this.apriltagTime = apriltagTime;
      this.singleTag = singleTag;
    }

    public static Result[] empty() {
      return new Result[] { new Result(null, null, 0.0, true) };
    }
  }
}