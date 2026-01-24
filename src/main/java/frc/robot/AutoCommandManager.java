// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.drive.Speed;
// import frc.robot.subsystems.vision.LimeLight4;
import frc.robot.subsystems.drive.SpeedProcessor;

/** Add your docs here. */
public class AutoCommandManager {
    public HolonomicDriveController holonomicDriveController;
    SendableChooser<Command> m_chooser;

    public static boolean isSim;

    public TrajectoryConfig forwardConfig;
    public TrajectoryConfig reverseConfig;

    public AutoCommandManager(DriveSubsystem m_robotDrive, SpeedProcessor m_speedProcessor) {
        configureAuto(m_robotDrive, m_speedProcessor);
        configureNamedCommands(m_robotDrive, m_speedProcessor);


      m_chooser = AutoBuilder.buildAutoChooserWithOptionsModifier(((p) -> p.filter((a) -> a.getName().startsWith("!"))));

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0,
            new TrapezoidProfile.Constraints(AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
                AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicDriveController = new HolonomicDriveController(
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController);

        forwardConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveConstants.kDriveKinematics);

        reverseConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, 
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(SwerveConstants.kDriveKinematics)
            .setReversed(true);

        isSim = true;

        m_chooser.setDefaultOption("None", new InstantCommand());

        SmartDashboard.putData(m_chooser);
    }

    public SendableChooser<Command> getChooser() {
        return m_chooser;
    }

    public Command getAutoManagerSelected() {
        return m_chooser.getSelected();
    }

    public SwerveControllerCommand trajectoryCommand(Trajectory trajectory, DriveSubsystem m_robotDrive) {
        return new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose,
            SwerveConstants.kDriveKinematics,
            holonomicDriveController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
    }

    public void configureAuto(DriveSubsystem m_robotDrive, SpeedProcessor m_speedProcessor) {
        // Configure the AutoBuilder last
        try {
            AutoBuilder.configure(
                m_robotDrive::getEstimatedPose, // Robot pose supplier
                m_robotDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                m_robotDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speed) -> m_speedProcessor.process(new Speed(speed, false)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(10.0, 0.0, 0.0)), // Rotation PID constants
                RobotConfig.fromGUISettings(),
                ()-> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_robotDrive
                // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
        // Reference: https://www.chiefdelphi.com/t/has-anyone-gotten-pathplanner-integrated-with-the-maxswerve-template/443646
    }

    public void configureNamedCommands(
        // LimeLight4 m_visionGS,
        DriveSubsystem m_robotDrive,
        SpeedProcessor m_speedProcessor
    ) {
        NamedCommands.registerCommand("AutoLineUp", Commands.runOnce(() -> m_speedProcessor.getContext().isAutoRotate = RotationEnum.STRAFEONTARGET));
        NamedCommands.registerCommand("LeftPole", Commands.runOnce(() -> m_robotDrive.leftPoint = true));
        NamedCommands.registerCommand("RightPole", Commands.runOnce(() -> m_robotDrive.leftPoint = false));
    }
}