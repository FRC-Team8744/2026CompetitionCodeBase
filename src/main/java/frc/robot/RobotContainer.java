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
import frc.robot.commands.RemainShooting;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // private final Rotation3d cameraToRobotOffsetRotationLeft = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(19.6), Units.degreesToRadians(-139.8));
  // private final Rotation3d cameraToRobotOffsetRotationRight = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20.2), Units.degreesToRadians(141.7));
  // private final Rotation3d cameraToRobotOffsetRotationLeft = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(19.6), Units.degreesToRadians(-147.4));
  // private final Rotation3d cameraToRobotOffsetRotationRight = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20.2), Units.degreesToRadians(146.0));
  private final Rotation3d cameraToRobotOffsetRotationLeft = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(19.6), Units.degreesToRadians(-147.4));
  private final Rotation3d cameraToRobotOffsetRotationRight = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(20.9), Units.degreesToRadians(146.0));

  // private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-10.59), Units.inchesToMeters(5.474), Units.inchesToMeters(14.142), cameraToRobotOffsetRotationLeft);
  // private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-10.59), Units.inchesToMeters(-5.474), Units.inchesToMeters(14.142), cameraToRobotOffsetRotationRight);
  // private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-8.705), Units.inchesToMeters(-6.942), Units.inchesToMeters(11.202), cameraToRobotOffsetRotationRight);
  // private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-7.25), Units.inchesToMeters(8.75), Units.inchesToMeters(15), cameraToRobotOffsetRotationLeft);
  // private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-7.25), Units.inchesToMeters(-8.75), Units.inchesToMeters(15), cameraToRobotOffsetRotationRight);
  private final Transform3d cameraToRobotOffsetLeft = new Transform3d(Units.inchesToMeters(-8.75), Units.inchesToMeters(0), Units.inchesToMeters(15), cameraToRobotOffsetRotationLeft);
  private final Transform3d cameraToRobotOffsetRight = new Transform3d(Units.inchesToMeters(-8.75), Units.inchesToMeters(0), Units.inchesToMeters(15), cameraToRobotOffsetRotationRight);
  // private final Transform3d cameraToRobotOffsetLeft = new Transform3d(0, 0, 0, cameraToRobotOffsetRotationLeft);
  // private final Transform3d cameraToRobotOffsetRight = new Transform3d(0, 0, 0, cameraToRobotOffsetRotationRight);

  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2026RebuiltAndymark.loadAprilTagLayoutField();
  // private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4.1", cameraToRobotOffset1), new PhotonVision.CameraWithOffsets("Limelight4.2", cameraToRobotOffset2));
  private final PhotonVision.Context photonVisionContext = new PhotonVision.Context(aprilTagFieldLayout, new PhotonVision.CameraWithOffsets("Limelight4Left", cameraToRobotOffsetLeft), new PhotonVision.CameraWithOffsets("Limelight4Right", cameraToRobotOffsetRight));
  private final PhotonVision m_visionPV = new PhotonVision(photonVisionContext);
  
  private final AlignToHub m_alignToHub = new AlignToHub();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_visionPV, m_alignToHub);
  // private final Climber m_climber = new Climber();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  private final IntakePivot m_intakePivot = new IntakePivot();
  private final ShooterFlywheels m_shooterFlywheels = new ShooterFlywheels();
  private final Spindexer m_spindexer = new Spindexer();
  private final Turret m_turret = new Turret(m_robotDrive);
  private final ShooterHood m_shooterHood = new ShooterHood(m_robotDrive, m_turret);
  private final ShooterHoodToZero m_shooterHoodToZero = new ShooterHoodToZero(m_shooterHood);
  private final RemainShooting m_remainShooting = new RemainShooting(m_intake, m_intakePivot, m_turret, m_indexer, m_shooterFlywheels, m_shooterHood, m_spindexer, m_shooterHoodToZero);
  // The driver's controller
  private final CommandXboxController m_driver = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_codriver = new CommandXboxController(OIConstants.kCoDriverControllerPort);
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
                      true,
                      false),
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
   m_driver.back().onTrue(Commands.runOnce (() -> m_robotDrive.zeroGyro()));
    // m_driver.b()
    // .whileTrue(Commands.runOnce(() -> m_robotDrive.isAutoYSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoXSpeed = false).alongWith(Commands.runOnce(() -> m_robotDrive.isAutoRotate = RotationEnum.NONE))));

    m_driver.leftTrigger()
    .whileTrue(new IntakeAndShootCommand(m_intake, m_intakePivot, m_turret, m_indexer, m_shooterFlywheels, m_shooterHood, m_spindexer, m_shooterHoodToZero, m_remainShooting));
    m_driver.rightTrigger()
    .whileTrue(new ShootCommand(m_shooterHood, m_spindexer, m_shooterFlywheels, m_indexer, m_turret, m_shooterHoodToZero, m_intake, m_intakePivot)
    .alongWith(Commands.runOnce(() -> m_robotDrive.m_DriverSpeedScale = .25)))
    .whileFalse(Commands.runOnce(() -> m_robotDrive.m_DriverSpeedScale = 1.0));

    m_driver.leftBumper()
    .whileTrue(new ToggleShuttleMode(m_turret, m_shooterFlywheels, m_shooterHoodToZero, m_spindexer, m_indexer, m_intake));
    m_driver.rightBumper()
    .whileTrue(new ToggleShootWhileIntakeMode(m_turret, m_shooterFlywheels, m_shooterHoodToZero, m_spindexer, m_indexer, m_intake));

    m_driver.pov(0)
    .whileTrue(Commands.runOnce(() -> m_intakePivot.intakeDown(-1600)));
    m_driver.pov(90)
    .whileTrue(Commands.runOnce(() -> m_indexer.setIndexerSpeed(-1)))
    .whileFalse(Commands.runOnce(() -> m_indexer.stopIndexer()));
    m_driver.pov(180)
    .whileTrue(Commands.runOnce(() -> m_spindexer.setSpindexerSpeed(1.0)))
    .whileFalse(Commands.runOnce(() -> m_spindexer.stopSpindexer()));
    // m_driver.pov(270)
    // .whileTrue(Commands.runOnce(() -> Constants.enableAntiStall = !Constants.enableAntiStall));
    m_driver.pov(270)
    .whileTrue(Commands.runOnce(() -> m_indexer.setIndexerSpeed(1.0)))
    .whileFalse(Commands.runOnce(() -> m_indexer.stopIndexer()));

    m_driver.x()
    .whileTrue(Commands.runOnce(() -> m_intake.setIntakeSpeed(-0.7)))
    .whileFalse(Commands.runOnce(() -> m_intake.stopIntake()));
    m_driver.b()
    .whileTrue(Commands.runOnce(() -> m_turret.setTurretAngle(180)));
    m_driver.a()
    .whileTrue(Commands.runOnce(() -> Constants.visionShoot = !Constants.visionShoot));
    // m_driver.y()
    // .whileTrue(Commands.runOnce(() -> Constants.enableAntiStall = !Constants.enableAntiStall));
    m_driver.y()
    .whileTrue(Commands.runOnce(() -> m_shooterHood.setShooterHoodAngle(60)))
    .whileFalse(Commands.runOnce(() -> m_shooterHood.setShooterHoodAngle(70)));

    m_codriver.pov(90)
    .whileTrue(Commands.runOnce(() -> Constants.targetShuttleRelativePosition = "Close"));
    m_codriver.pov(270)
    .whileTrue(Commands.runOnce(() -> Constants.targetShuttleRelativePosition = "Far"));
    // m_codriver.pov(0)
    // .whileTrue(new ParallelCommandGroup(Commands.runOnce(() -> Constants.presetFlywheelSpeed = 70)));
    // m_codriver.pov(180)
    // .whileTrue(new ParallelCommandGroup(Commands.runOnce(() -> Constants.presetFlywheelSpeed = 40)));
    m_codriver.pov(0)
    .whileTrue(Commands.runOnce(() -> Constants.presetFlywheelSpeed += 1));
    m_codriver.pov(180)
    .whileTrue(Commands.runOnce(() -> Constants.presetFlywheelSpeed -= 1));
    m_codriver.y()
    .whileTrue(Commands.runOnce(() -> Constants.presetFlywheelSpeed = 70));
    m_codriver.b()
    .whileTrue(Commands.runOnce(() -> Constants.presetFlywheelSpeed = 60));
    m_codriver.a()
    .whileTrue(Commands.runOnce(() -> Constants.presetFlywheelSpeed = 50));
    m_codriver.x()
    .whileTrue(Commands.runOnce(() -> Constants.presetFlywheelSpeed = 40));
    m_codriver.pov(90)
    .whileTrue(Commands.runOnce(() -> m_shooterFlywheels.stopShooterFlywheels()));
    // m_driver.a()

    m_driver.rightStick()
    .toggleOnTrue(Commands.runOnce(() -> Constants.isAutoRotate = Constants.isAutoRotate == RotationEnum.ALIGNTOHUB ? RotationEnum.NONE : RotationEnum.ALIGNTOHUB));
  }

  public Command getAutonomousCommand() {
    Command autoCommand = m_autoManager.getAutoManagerSelected();
    return autoCommand;
  }
}