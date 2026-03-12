// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.utils.RobotHealth;
import frc.robot.utils.Telemetry;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public DriveSubsystem m_RobotDrive = new DriveSubsystem();
  public Intake m_Intake = new Intake();
  public Launcher m_Launcher = new Launcher(m_RobotDrive);
  public Extender m_Extender = new Extender();
  public Feeder m_Feeder = new Feeder();

  // The driver's controller
  Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
  Joystick m_operatorJoystick = new Joystick(OIConstants.kOperatorJoystickPort);

  JoystickButton b_IntakeReceive = new JoystickButton(m_operatorJoystick, OIConstants.kIntakeReceiveButton);
  JoystickButton b_ExtendOut = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderOutButton);
  JoystickButton b_ExtendIn = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderInButton);
  JoystickButton b_Launcher = new JoystickButton(m_operatorJoystick, OIConstants.kLauncherButton);
  JoystickButton b_LauncherIdleOn = new JoystickButton(m_operatorJoystick, OIConstants.kLauncherIdleOnButton);
  JoystickButton b_LauncherIdleOff = new JoystickButton(m_operatorJoystick, OIConstants.kLauncherIdleOffButton);
  JoystickButton b_HomeExtender = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderHomeButton);
  JoystickButton b_OrientRobot = new JoystickButton(m_driverJoystick, OIConstants.kOrientRobot);
  JoystickButton b_AlignRobot = new JoystickButton(m_driverJoystick, OIConstants.kAlignRobot);
  JoystickButton b_LaunchModeToggle = new JoystickButton(m_operatorJoystick, OIConstants.kSwitchLaunchMode);
  JoystickButton b_ToggleDriveMode = new JoystickButton(m_driverJoystick, OIConstants.kToggleDriveMode);
  JoystickButton b_ExtendStopFollow = new JoystickButton(m_operatorJoystick, OIConstants.kExtenderFollowJoystick);
  JoystickButton b_EmergencyStop = new JoystickButton(m_operatorJoystick, OIConstants.kEmergencyStop); // B button
  JoystickButton b_ClearExtenderFaults = new JoystickButton(m_operatorJoystick, OIConstants.kClearExtenderFaults); // X
                                                                                                                   // button

  boolean fieldRelative = true;
  boolean useVisionForLaunch = true;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Enable debug telemetry by default.
    Telemetry.setDebugEnabled(true);

    // Configure the trigger bindings
    configureBindings();

    m_RobotDrive.setDefaultCommand(
        m_RobotDrive.driveCommand(
            () -> m_driverJoystick.getY(),
            () -> m_driverJoystick.getX(),
            () -> m_driverJoystick.getZ(),
            () -> fieldRelative));
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {

    this.b_EmergencyStop.onTrue(stopAll());
    this.b_ClearExtenderFaults.onTrue(m_Extender.clearFaultsCommand());

    this.b_ToggleDriveMode.onTrue(Commands.runOnce(() -> fieldRelative = !fieldRelative));

    this.b_LaunchModeToggle.onTrue(
        Commands.runOnce(() -> useVisionForLaunch = !useVisionForLaunch));

    // bring in the extender and stop the intake
    this.b_ExtendIn
      .and(() -> RobotHealth.isHealthy("Extender Leader"))
      .and(() -> RobotHealth.isHealthy("Extender Follower"))
      .onTrue(m_Extender.stow().alongWith(m_Intake.stop()));

    // send the extender out
    this.b_ExtendOut
        .and(() -> RobotHealth.isHealthy("Extender Leader"))
        .and(() -> RobotHealth.isHealthy("Extender Follower"))
        .onTrue(m_Extender.fullExtend());

    this.b_ExtendStopFollow
        .and(() -> RobotHealth.isHealthy("Extender Leader"))
        .and(() -> RobotHealth.isHealthy("Extender Follower"))
        .onTrue(m_Extender.stopCommand()) // Stop immediately when pressed
        .whileTrue(m_Extender.followJoystick(
            () -> m_driverJoystick.getX() // Pass the joystick supplier
        ));

    this.b_HomeExtender
      .and(() -> RobotHealth.isHealthy("Extender Leader"))
      .and(() -> RobotHealth.isHealthy("Extender Follower"))
      .onTrue(m_Extender.home());

     // turn on the intake motor to pick up the fuel cells
    this.b_IntakeReceive
      .and(() -> RobotHealth.isHealthy("Intake"))
      .whileTrue(m_Intake.receive());

    // set launcher to idle
    this.b_LauncherIdleOn
      .and(() -> RobotHealth.isHealthy("Launcher"))
      .and(() -> RobotHealth.isHealthy("Feeder"))
      .onTrue(m_Launcher.idle());

    // turn off idle
    this.b_LauncherIdleOff
      .and(() -> RobotHealth.isHealthy("Launcher"))
      .and(() -> RobotHealth.isHealthy("Feeder"))
      .onTrue(m_Launcher.off());

    // Unified Launch Command: Align, Spool based on distance, and Feed when ready
    this.b_Launcher
        .and(() -> RobotHealth.isHealthy("Launcher"))
        .and(() -> RobotHealth.isHealthy("Feeder"))
        .whileTrue(
                Commands.either(
                    m_Launcher.align(() -> m_driverJoystick.getY(), () -> m_driverJoystick.getX(), () -> false)
                        .alongWith(
                            m_Launcher.launchWithVision()),
                    m_Launcher.launchWithJoystick(() -> m_operatorJoystick.getRawAxis(5)),
                    () -> useVisionForLaunch
                )
                .alongWith(
                    Commands.waitUntil(() -> m_Launcher.atSpeed() && (!useVisionForLaunch || m_Launcher.isAligned()))
                        .andThen(m_Feeder.feedLauncher())));
    
    // orient the robot to the field
    this.b_OrientRobot.onTrue(m_RobotDrive.orient());

    this.b_AlignRobot.whileTrue(m_Launcher.align(() -> m_driverJoystick.getY(), () -> m_driverJoystick.getX(), () -> fieldRelative));

  }

  public void startHealthChecks() {
    // 0.5 means it checks every half-second.
    healthCheckNotifier.startPeriodic(0.5);
  }

  private final Notifier healthCheckNotifier = new Notifier(() -> {
    // This code runs in the BACKGROUND. No lag for you!
    checkMotor(m_RobotDrive.getRightFrontDrive(), "Right Front Drive");
    checkMotor(m_RobotDrive.getRightFrontTurn(), "Right Front Turn");
    checkMotor(m_RobotDrive.getLeftFrontDrive(), "Left Front Drive");
    checkMotor(m_RobotDrive.getLeftFrontTurn(), "Left Front Turn");
    checkMotor(m_RobotDrive.getRightRearDrive(), "Right Rear Drive");
    checkMotor(m_RobotDrive.getRightRearTurn(), "Right Rear Turn");
    checkMotor(m_RobotDrive.getLeftRearDrive(), "Left Rear Drive");
    checkMotor(m_RobotDrive.getLeftRearTurn(), "Left Rear Turn");
    checkMotor(m_Extender.getLeader(), "Extender Leader");
    checkMotor(m_Extender.getFollower(), "Extender Follower");
    checkMotor(m_Intake.getMotor(), "Intake");
    checkMotor(m_Feeder.getMotor(), "Feeder");
    checkMotor(m_Launcher.getMotor(), "Launcher");
    // This sends the big green/red light signal
    Telemetry.putBoolean("Health/All Systems Good", RobotHealth.isEverythingOk());
  });

  private void checkMotor(SparkMax motor, String name) {
    // 1. Connection check (firmware version 0 means not reachable)
    if (motor.getFirmwareVersion() == 0) {
      updateMotorStatus(name, motor.getDeviceId(), "ERROR (DISCONNECTED)");
      return;
    }

    // 2. Fault check
    var faults = motor.getFaults();
    if (motor.hasActiveFault()) {
      String faultDescription = "ERROR (";
      if (faults.sensor)
        faultDescription += "SENSOR ";
      if (faults.temperature)
        faultDescription += "HOT ";
      faultDescription += ")";
      updateMotorStatus(name, motor.getDeviceId(), faultDescription);
    } else {
      updateMotorStatus(name, motor.getDeviceId(), "OK");
    }
  }

  private void checkMotor(TalonFX motor, String name) {
    // 1. Connection check
    var version = motor.getVersion();
    if (!version.getStatus().isOK()) {
      updateMotorStatus(name, motor.getDeviceID(), "ERROR (DISCONNECTED)");
      return;
    }

    // 2. Thermal check
    var temp = motor.getDeviceTemp();
    if (temp.getValueAsDouble() > 70.0) {
      updateMotorStatus(name, motor.getDeviceID(), "WARNING (OVERHEATING)");
    } else {
      updateMotorStatus(name, motor.getDeviceID(), "OK");
    }
  }

  private void updateMotorStatus(String name, int motorId, String status) {
    RobotHealth.updateStatus(name, status);
    Telemetry.putString("Health/" + name + " (" + motorId + ")", status);
  }

  /**
   * Stops all mechanisms on the robot immediately.
   */
  public Command stopAll() {
    return Commands.parallel(
        m_RobotDrive.driveCommand(() -> 0, () -> 0, () -> 0, () -> false),
        m_Intake.stop(),
        m_Launcher.off(),
        m_Extender.stopCommand(),
        m_Feeder.feedLauncher().withTimeout(0)).withName("EmergencyStop");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            m_Extender.partialExtend(),
            m_Launcher.launchWithVision()
                .alongWith(
                    new WaitUntilCommand(m_Launcher::atSpeed)
                        .andThen(m_Feeder.feedLauncher().withTimeout(10.0)))),
        m_Extender.fullExtend());
  }

  public void periodic() {
    Telemetry.putBoolean("Launcher/Launch With Vision", useVisionForLaunch);
    Telemetry.putBoolean("Drive/Field Oriented", fieldRelative);
  }
}
