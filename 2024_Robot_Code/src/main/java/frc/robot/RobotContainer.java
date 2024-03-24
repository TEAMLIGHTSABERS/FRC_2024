// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final TurretSubsystem m_turret = new TurretSubsystem();
  private static final ExampleSubsystem ExampleSubsystem = new ExampleSubsystem();
//  private ProfiledPIDController lastThetaController;
//  private PIDController lastXAxisController;
//  private PIDController lastYAxisController;

  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command m_simpleAuto = Autos.exampleAuto(ExampleSubsystem);

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_complexAuto = sTurnAutoCommand();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();

    SmartDashboard.putData("Launcher Wheel Test", m_launcher.testFlyWheels());
 
    // Add Button to SmartDashboard with a RunCommand that executes a "void" method/function.
    SmartDashboard.putData("LUp-1000", 
      new RunCommand(() -> m_launcher.raiseLCR1000(), m_launcher));

    // Add Buttons to the SmartDashboard that executes a "command" methods/functions.
    SmartDashboard.putData("LUp-100", m_launcher.raiseLCR100());
    SmartDashboard.putData("LDn-1000", m_launcher.downLC1000());
    SmartDashboard.putData("LDn-100", m_launcher.downLC100());
    SmartDashboard.putData("RUp-1000", m_launcher.raiseRC1000());
    SmartDashboard.putData("RDn-1000", m_launcher.downRC1000());
    SmartDashboard.putData("RUp-100", m_launcher.raiseRC100());
    SmartDashboard.putData("RDn-100", m_launcher.downRC100());
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((m_driverController).getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController).getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((m_driverController).getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    // configure the launcher to stop when no other command is running
    m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));

    m_turret.setDefaultCommand(new RunCommand(() -> m_turret.driveWench(
        ((m_driverController).getRightTriggerAxis() > Constants.OIConstants.kTriggerButtonThreshold),
        (m_driverController).getRightBumperPressed()),
        m_turret));

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);
    SmartDashboard.putData("Auto Selection", m_chooser);
//    SmartDashboard.putData(m_launcher);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

    // Put subsystems to dashboard.
    Shuffleboard.getTab("Drivetrain").add("Commands", m_robotDrive);
    Shuffleboard.getTab("Intake Subsystem").add("Commands", m_intake);
    Shuffleboard.getTab("Launcher Subsystem").add("Commands", m_launcher);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // button to put swerve modules in an "x" configuration to hold position
    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    // Turret Controls ----------------------------------------------------------
    // Right Trigger will Raise the Launcher elevation.    
//    new Trigger(
//            () -> ((XboxController) m_driverController).getRightTriggerAxis()
//            > Constants.OIConstants.kTriggerButtonThreshold)
///        .onTrue(new RunCommand(() -> m_turret.reducePOS()));

    // Right Bumper will Lower the Launcher elevation
//    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
//        .onTrue(new RunCommand(() -> m_turret.advancePOS()));

    // Intake Controls ----------------------------------------------------------
    // Left Trigger will start the Intake and pickup a Note
    new Trigger(
            () ->
                ((XboxController) m_driverController).getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kTopPower, Constants.Intake.kFeedPower), m_intake))
        .onFalse(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    // "B" Button will slowly "Backup" a Note in the Intake
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(0.0, -0.2), m_intake))
        .onFalse(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    // Launcher Controls -------------------------------------------------------
    // "A" Button will launch a Note toward the Target
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(m_launcher.launchNote(m_intake));
    
    // "X" Button will run the Launcher Flywheels for 5 seconds
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(m_launcher.testFlyWheels());
  }

 /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public Command sTurnAutoCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    edu.wpi.first.math.trajectory.Trajectory sTurnTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xAxisController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yAxisController = new PIDController(AutoConstants.kPYController, 0, 0);

    SendableRegistry.setName(thetaController, "Drive Subsystem", "Turret Azimuth");
    SendableRegistry.setName(xAxisController, "Drive Subsystem", "X Axis");
    SendableRegistry.setName(yAxisController, "Drive Subsystem", "Y Axis");

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    sTurnTrajectory,
    m_robotDrive::getPose, // Functional interface to feed supplier
    DriveConstants.kDriveKinematics,

    // Position controllers
    xAxisController,
    yAxisController,
    thetaController,
    m_robotDrive::setModuleStates,
    m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(sTurnTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

}
