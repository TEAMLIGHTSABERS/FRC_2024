// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  // A simple auto routine that drives forward a specified distance, and then stops.
  private final Command m_CenterAuto = Autos.CenterAutoCommand(
    m_robotDrive,
    m_launcher,
    m_intake
  );

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command m_complexAuto = Autos.sTurnAutoCommand(m_robotDrive);

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

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Center Auto", m_CenterAuto);
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

    // intake controls
    new Trigger(
            () ->
                ((XboxController) m_driverController).getLeftTriggerAxis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kTopPower, Constants.Intake.kFeedPower), m_intake))
        .onFalse(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> m_intake.setPower(0.0, -0.2), m_intake))
        .onFalse(new RunCommand(() -> m_intake.setPower(0.0, 0.0), m_intake));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(m_launcher.launchNote(m_intake));
        
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

}
