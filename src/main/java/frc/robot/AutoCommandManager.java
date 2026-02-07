// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.vision.LimeLight4;

/** Add your docs here. */
public class AutoCommandManager {
    public HolonomicDriveController holonomicDriveController;
    SendableChooser<Command> m_chooser;

    public static boolean isSim;

    public TrajectoryConfig forwardConfig;
    public TrajectoryConfig reverseConfig;

    public AutoCommandManager(
        // LimeLight4 m_visionGS,
        DriveSubsystem m_robotDrive)
         {

        configureNamedCommands(
            // m_visionGS,
            m_robotDrive
      );

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

    public void configureNamedCommands(
        // LimeLight4 m_visionGS,
        DriveSubsystem m_robotDrive
    ) {
        // TODO: Add named commands for auto builder to use
        NamedCommands.registerCommand("AutoLineUp", Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.STRAFEONTARGET));
        NamedCommands.registerCommand("LeftPole", Commands.runOnce(() -> m_robotDrive.leftPoint = true));
        NamedCommands.registerCommand("RightPole", Commands.runOnce(() -> m_robotDrive.leftPoint = false));
    }
}