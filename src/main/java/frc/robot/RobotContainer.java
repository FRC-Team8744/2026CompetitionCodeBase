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
import frc.robot.commands.IntakeAndShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterHoodToZero;
import frc.robot.commands.ToggleShootWhileIntakeMode;
import frc.robot.commands.ToggleShuttleMode;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.alignment.AlignToHub;
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
  private final Rotation3d cameraToRobotOffsetRotationLeft = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(-148.0));
  private final Rotation3d cameraToRobotOffsetRotationRight = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20), Units.degreesToRadians(148.0));

  // private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-10.59), Units.inchesToMeters(5.474), Units.inchesToMeters(14.142), cameraToRobotOffsetRotationLeft);
  // private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-10.59), Units.inchesToMeters(-5.474), Units.inchesToMeters(14.142), cameraToRobotOffsetRotationRight);
  private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-8.705), Units.inchesToMeters(6.942), Units.inchesToMeters(11.202), cameraToRobotOffsetRotationLeft);
  private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-8.705), Units.inchesToMeters(-6.942), Units.inchesToMeters(11.202), cameraToRobotOffsetRotationRight);

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
  // private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4.1", cameraToRobotOffset1), new PhotonVision.CameraWithOffsets("Limelight4.2", cameraToRobotOffset2));
  private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4Left", cameraToRobotOffsetLeft), new PhotonVision.CameraWithOffsets("Limelight4Right", cameraToRobotOffsetRight));
  private final PhotonVision m_visionPV = new PhotonVision(photonVisionContext);
  
  private final AlignToHub m_alignToHub = new AlignToHub();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionPV, m_alignToHub);
  // TODO: Add other subsystems
  // private final Climber m_climber = new Climber();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  private final IntakePivot m_intakePivot = new IntakePivot();
  private final ShooterFlywheels m_shooterFlywheels = new ShooterFlywheels();
  private final Spindexer m_spindexer = new Spindexer();
  private final Turret m_turret = new Turret(m_robotDrive);
  private final ShooterHood m_shooterHood = new ShooterHood(m_robotDrive, m_turret);
  private final ShooterHoodToZero m_shooterHoodToZero = new ShooterHoodToZero(m_shooterHood);
  // The driver's controller
  private final CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  // private CommandXboxController m_coDriver = new CommandXboxController(1);
  private final AutoCommandManager m_autoManager = new AutoCommandManager(m_robotDrive, m_shooterFlywheels, m_shooterHood, m_intake, m_indexer, m_spindexer, m_intakePivot, m_turret);
  
  
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
                      -m_driver.getRightX(),
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
    // m_driver.b()
    // .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoXSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.NONE))));

    m_driver.leftTrigger()
    .whileTrue(new IntakeAndShootCommand(m_intake, m_intakePivot, m_turret, m_indexer, m_shooterFlywheels, m_shooterHood, m_spindexer));
    m_driver.rightTrigger()
    .whileTrue(new ShootCommand(m_shooterHood, m_spindexer, m_shooterFlywheels, m_indexer, m_turret, m_shooterHoodToZero, m_intake, m_intakePivot));

    m_driver.leftBumper()
    .whileTrue(new ToggleShuttleMode(m_turret, m_shooterFlywheels, m_shooterHoodToZero));
    m_driver.rightBumper()
    .whileTrue(new ToggleShootWhileIntakeMode(m_turret, m_shooterFlywheels, m_shooterHoodToZero));

    m_driver.pov(0)
    .whileTrue(Commands.run(() -> m_intakePivot.intakeDown(0)));
    m_driver.pov(90)
    .whileTrue(Commands.run(() -> m_indexer.setIndexerSpeed(-1)))
    .whileFalse(Commands.run(() -> m_indexer.stopIndexer()));
    m_driver.pov(180)
    .whileTrue(Commands.runOnce(() -> m_shooterHood.setHoodRollerSpeed(0.1)))
    .whileFalse(Commands.runOnce(() -> m_shooterHood.stopHoodRollers()));
    // m_driver.pov(270)

    m_driver.x()
    .whileTrue(Commands.runOnce(() -> Constants.intakeSpeed = -0.7))
    .whileFalse(Commands.runOnce(() -> Constants.intakeSpeed = 0.7));
    m_driver.b()
    .whileTrue(Commands.runOnce(() -> m_turret.setTurretAngle(180)));
    // m_driver.a()

    m_driver.rightStick()
    .toggleOnTrue(Commands.runOnce(() -> Constants.isAutoRotate = Constants.isAutoRotate == RotationEnum.ALIGNTOHUB ? RotationEnum.NONE : RotationEnum.ALIGNTOHUB));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}