// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.alignment.AlignToPoleX;
import frc.robot.subsystems.drive.DriveContext;
import frc.robot.subsystems.drive.RunSlow;
import frc.robot.subsystems.drive.Speed;
import frc.robot.subsystems.drive.SpeedProcessor;
import frc.robot.subsystems.drive.Teleop;
import frc.robot.subsystems.vision.PhotonVision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // private LimeLight4 m_vision = new LimeLight4();
  private PhotonVision m_visionPV = new PhotonVision();
  // private Limelight4Test m_limelight4Test = new Limelight4Test();
    // The driver's controller
  private CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  private Teleop m_teleop = new Teleop(
      OIConstants.kDeadband,
      () -> -m_driver.getLeftY() * SwerveConstants.kMaxSpeedTeleop,
      () -> -m_driver.getLeftX() * SwerveConstants.kMaxSpeedTeleop,
      () -> m_driver.getRightX() * ConstantsOffboard.MAX_ANGULAR_RADIANS_PER_SECOND
  );
  private AlignToPoleX m_alignToPoleX = new AlignToPoleX();
  private RunSlow m_runSlow = new RunSlow(new Speed(0.1, 0.1, 1));
  private DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionPV);
  private DriveContext m_driveContext = new DriveContext(m_robotDrive);
  private SpeedProcessor m_speedProcessor = new SpeedProcessor(m_driveContext, m_teleop, m_alignToPoleX, m_runSlow);
  // private CommandXboxController m_coDriver = new CommandXboxController(1);
  private AutoCommandManager m_autoManager = new AutoCommandManager(m_robotDrive, m_speedProcessor);
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
      m_robotDrive.setDefaultCommand(new RunCommand(() -> m_speedProcessor.process(), m_robotDrive));
    // m_autoChooser = AutoBuilder.buildAutoChooser();  // Default auto will be 'Commands.none()'

    // SmartDashboard.putData("Auto Mode", m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  
  private void configureButtonBindings() {
    m_driver.back().onTrue(Commands.runOnce (() -> m_robotDrive.zeroGyro()));
    m_driver.rightStick()
    .toggleOnTrue(Commands.runOnce(() -> m_driveContext.isAutoRotate = m_driveContext.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));

    m_driver.b()
    .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false).alongWith(Commands.runOnce(() -> m_driveContext.isAutoXSpeed = false).alongWith(Commands.runOnce(() -> m_driveContext.isAutoRotate = RotationEnum.NONE))));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}