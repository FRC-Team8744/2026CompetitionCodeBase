// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ConstantsOffboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.mechanisms.Climber;
import frc.robot.subsystems.mechanisms.Indexer;
import frc.robot.subsystems.mechanisms.Intake;
import frc.robot.subsystems.mechanisms.IntakePivot;
import frc.robot.subsystems.mechanisms.ShooterFlywheels;
import frc.robot.subsystems.mechanisms.ShooterHood;
import frc.robot.subsystems.mechanisms.Spindexer;
import frc.robot.subsystems.mechanisms.Turret;
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
  // TODO: Add new offsets for the cameras
  private final Rotation3d cameraToRobotOffsetRotationLeft = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-130.0));
  private final Rotation3d cameraToRobotOffsetRotationRight = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(130.0));

  private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-10.59), Units.inchesToMeters(5.474), Units.inchesToMeters(14.142), cameraToRobotOffsetRotationLeft);
  private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-10.59), Units.inchesToMeters(-5.474), Units.inchesToMeters(14.142), cameraToRobotOffsetRotationRight);

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
  // private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4.1", cameraToRobotOffset1), new PhotonVision.CameraWithOffsets("Limelight4.2", cameraToRobotOffset2));
  private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4Left", cameraToRobotOffsetLeft), new PhotonVision.CameraWithOffsets("Limelight4Right", cameraToRobotOffsetRight));
  private final PhotonVision m_visionPV = new PhotonVision(photonVisionContext);
  // private Limelight4Test m_limelight4Test = new Limelight4Test();0
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionPV);
  // TODO: Add other subsystems
  // private final Climber m_climber = new Climber();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  private final IntakePivot m_intakePivot = new IntakePivot();
  private final ShooterFlywheels m_shooterFlywheels = new ShooterFlywheels();
  private final ShooterHood m_shooterHood = new ShooterHood();
  private final Spindexer m_spindexer = new Spindexer();
  private final Turret m_turret = new Turret();
  // The driver's controller
  private final CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  // private CommandXboxController m_coDriver = new CommandXboxController(1);
  private final AutoCommandManager m_autoManager = new AutoCommandManager(m_robotDrive);
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // Configure the button bindings
    configureButtonBindings();

  // Configure default commands
      m_robotDrive.setDefaultCommand(
          // The left stick controls translation of the robot.
          // Turning is controlled by the X axis of the right stick.
          new RunCommand(
              () ->
                  m_robotDrive.drive(
                      -m_driver.getLeftY(),
                      -m_driver.getLeftX(),
                      m_driver.getRightX(),
                      true),
              m_robotDrive));
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
    // TODO: Add button bindings for the commands
    m_driver.back().onTrue(Commands.runOnce (() -> m_robotDrive.zeroGyro()));
    m_driver.rightStick()
    .toggleOnTrue(Commands.runOnce(() -> m_robotDrive.isAutoRotate = m_robotDrive.isAutoRotate == RotationEnum.STRAFEONTARGET ? RotationEnum.NONE : RotationEnum.STRAFEONTARGET));
    m_driver.a()
    .whileTrue(Commands.runOnce(() -> Constants.shuttleMode = !Constants.shuttleMode));
    m_driver.b()
    .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoXSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.NONE))));

    m_driver.leftTrigger()
    .whileTrue(new IntakeCommand(m_intake, m_intakePivot, m_turret));
    m_driver.leftBumper()
    .whileTrue(Commands.run(() -> m_spindexer.setSpindexerSpeed(-0.3)))
    .whileFalse(Commands.run(() -> m_spindexer.stopSpindexer()));
    m_driver.rightBumper()
    .whileTrue(Commands.run(() -> m_indexer.setIndexerSpeed(0.3)))
    .whileFalse(Commands.run(() -> m_indexer.stopIndexer()));
    m_driver.rightTrigger()
    .whileTrue(Commands.run(() -> m_shooterHood.setShooterHoodAngle(60)));
    m_driver.y()
    .whileTrue(Commands.runOnce(() -> m_shooterHood.setHoodRollerSpeed(0.2)))
    .whileFalse(Commands.runOnce(() -> m_shooterHood.stopHoodRollers()));
    m_driver.x()
    .whileTrue(Commands.runOnce(() -> m_shooterFlywheels.setShooterFlywheelsSpeed(1.0)))
    .whileFalse(Commands.runOnce(() -> m_shooterFlywheels.stopShooterFlywheels()));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}