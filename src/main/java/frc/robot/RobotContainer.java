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

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  public Launcher m_Launcher = new Launcher();
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

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_RobotDrive.setDefaultCommand(
            m_RobotDrive.driveCommand(
            () -> m_driverJoystick.getY(),
            () -> m_driverJoystick.getX(),
            () -> m_driverJoystick.getZ(),
            () -> false));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
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

    // set launcher speed based on the slider
    this.b_Launcher
      .and(() -> RobotHealth.isHealthy("Launcher"))
      .and(() -> RobotHealth.isHealthy("Feeder"))
      .whileTrue(
        m_Launcher.launch(() -> m_operatorJoystick.getRawAxis(5))
            .alongWith(
                // turn on the feeder after launcher has reached speed
                new WaitUntilCommand(m_Launcher::atSpeed)
                    .andThen(m_Feeder.feedLauncher())));
  }

  public void startHealthChecks() {
    // 0.5 means it checks every half-second.
    // Fast enough to see a failure, slow enough to not kill your CAN bus.
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
    
    // This sends one big string like "{Extender Leader=OK, Feeder=DISCONNECTED}"
    SmartDashboard.putString("Health/Report", RobotHealth.getReport());
    
    // This sends the big green/red light signal
    SmartDashboard.putBoolean("Health/All Systems Nominal", RobotHealth.isEverythingOk());
  });

  private void checkMotor(SparkMax motor, String name) {
    if (motor.getFirmwareVersion() == 0) {
      RobotHealth.updateStatus(name, "DISCONNECTED");
    } else {
      RobotHealth.updateStatus(name, "OK");
    }
  }

  private void checkMotor(TalonFX motor, String name) {
    // In Phoenix 6, we check the version of the firmware
    // If the device isn't on the bus, this will return an error code
    var version = motor.getVersion();

    if (!version.getStatus().isOK()) {
      RobotHealth.updateStatus(name, "DISCONNECTED/ERROR");
    } else {
      RobotHealth.updateStatus(name, "OK");
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
