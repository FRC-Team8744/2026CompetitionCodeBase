package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface Drivable {
    void drive(Speed initialSpeed);
    void driveRobotRelative(ChassisSpeeds speed);
    void driveFieldRelative(ChassisSpeeds speed);
    Pose2d getEstimatedPose();
}
