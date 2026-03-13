// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIdConstants;
import frc.robot.configs.LauncherConfigs;
import frc.robot.constants.LauncherConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.VisionUtils;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Telemetry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.util.Units;

public class Launcher extends SubsystemBase {
  private final TalonFX m_IntakeMotor;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final DriveSubsystem m_DriveSystem;

  private final PIDController m_AlignPID = new PIDController(0.04, 0, 0.001);
  private final LinearFilter m_TXFilter = LinearFilter.movingAverage(5);
  private final LinearFilter m_TYFilter = LinearFilter.movingAverage(5);

  private double m_TargetSpeed;
  private boolean m_IdleOn = false;
  private boolean m_HasTarget;
  private double m_TX;
  private double m_TY;
  NetworkTable m_Table;

  private final LoggedTunableNumber alignP = new LoggedTunableNumber("Launcher/Align/kP", LauncherConstants.kAlignP);
  private final LoggedTunableNumber alignD = new LoggedTunableNumber("Launcher/Align/kD", LauncherConstants.kAlignD);
  private final LoggedTunableNumber mountAngle = new LoggedTunableNumber("Vision/MountAngle", VisionConstants.kMountAngle);
  private final LoggedTunableNumber shotOffset = new LoggedTunableNumber("Launcher/ShotSpeedOffset", LauncherConstants.kShotOffset);
  
  private final LoggedTunableNumber flyP = new LoggedTunableNumber("Launcher/Flywheel/kP", LauncherConstants.kP);
  private final LoggedTunableNumber flyS = new LoggedTunableNumber("Launcher/Flywheel/kS", LauncherConstants.kS);
  private final LoggedTunableNumber flyV = new LoggedTunableNumber("Launcher/Flywheel/kV", LauncherConstants.kV);

  /** Creates a new Launcher. */
  public Launcher(DriveSubsystem driveSystem) {
    this.m_IntakeMotor = new TalonFX(CanIdConstants.kLauncherMotor);
    this.m_IntakeMotor.getConfigurator().apply(LauncherConfigs.config);
    this.m_DriveSystem = driveSystem;
    this.m_TX = 0.0;
    this.m_TY = 0.0;
    this.m_HasTarget = false;
    this.m_TargetSpeed = 0.0;
    this.m_Table = NetworkTableInstance.getDefault().getTable("limelight");
    
    m_AlignPID.setTolerance(1.0); // 1 degree tolerance
    
    // Ensure Limelight is in the correct pipeline for vision tracking
    this.m_Table.getEntry("pipeline").setNumber(VisionConstants.VISION_PIPELINE);
  }

  @Override
  public void periodic() {
    // Update tuning values if they changed on the dashboard
    LoggedTunableNumber.ifChanged(hashCode(), () -> {
      m_AlignPID.setP(alignP.get());
      m_AlignPID.setD(alignD.get());
    }, alignP, alignD);

    LoggedTunableNumber.ifChanged(hashCode() + 1, () -> {
      TalonFXConfiguration config = new TalonFXConfiguration();
      m_IntakeMotor.getConfigurator().refresh(config);
      config.Slot0.kP = flyP.get();
      config.Slot0.kS = flyS.get();
      config.Slot0.kV = flyV.get();
      m_IntakeMotor.getConfigurator().apply(config);
    }, flyP, flyS, flyV);

    // Check for target validity (tv == 1.0) and retrieve the target ID (tid).
    boolean hasTarget = m_Table.getEntry(VisionConstants.kTargetValidKey)
        .getDouble(VisionConstants.kDefaultTargetValid) == 1.0;
    int tagID = (int) m_Table.getEntry(VisionConstants.kTargetIdKey).getInteger(VisionConstants.kDefaultTargetId);

    // If a valid target is found and it's the correct hoop, calculate RPM.
    if (hasTarget && VisionUtils.isTargetingCorrectHoop(tagID)) {
      this.m_HasTarget = true;
      // Filter the vision inputs to reduce jitter
      m_TY = m_TYFilter.calculate(m_Table.getEntry(VisionConstants.kTargetYKey).getDouble(VisionConstants.kDefaultTargetY));
      m_TX = m_TXFilter.calculate(m_Table.getEntry(VisionConstants.kTargetXKey).getDouble(VisionConstants.kDefaultTargetX));
    } else {
      this.m_HasTarget = false;
      m_TY = 0.0;
      m_TX = 0.0;
      m_TXFilter.reset();
      m_TYFilter.reset();
    }

    Telemetry.putNumber("Launcher/Distance To Target", Units.metersToInches(getDistanceToTarget()));
    Telemetry.putDebugNumber("Launcher/Current Speed", getActualVelocity());
    Telemetry.putDebugNumber("Launcher/Target Speed", m_TargetSpeed);
    Telemetry.putBoolean("Launcher/At Speed", atSpeed());
    Telemetry.putBoolean("Launcher/Is Aligned", isAligned());

    // Debugging high-frequency vision data
    Telemetry.putDebugNumber("Launcher/Raw TX", m_TX);
    Telemetry.putDebugNumber("Launcher/Raw TY", m_TY);

  }

  private double calculateRPMFromSlider(double slideValue) {
    double min = LauncherConstants.kMinLaunchSpeed;
    double max = LauncherConstants.kMaxLaunchSpeed;

    double speed = min + (slideValue * (max - min));
    return speed;
  }

  private void runMotor(double speed) {
    this.m_IntakeMotor.setControl(m_velocityRequest.withVelocity(speed));
  }

  private void stopMotor() {
    this.m_IntakeMotor.stopMotor();
  }

  private double getActualVelocity() {
    return this.m_IntakeMotor.getVelocity().getValueAsDouble();
  }

  public boolean atSpeed() {
    // 1. Calculate if the actual RPM is within an acceptable tolerance of the
    // target RPM.
    boolean isNearTarget = Math.abs(this.m_TargetSpeed - getActualVelocity()) < LauncherConstants.kTolerance;

    // 2. Ensure the target speed is a "launch" speed (i.e., significantly above
    // idle),
    // to differentiate from idle state or a stopped state.
    boolean isNotIdle = this.m_TargetSpeed > (LauncherConstants.kLauncherIdleSpeed + LauncherConstants.kTolerance);

    return isNearTarget && isNotIdle;
  }

  public boolean isAligned() {
    return m_HasTarget && Math.abs(m_TX) < 1.0; // 1.0 degree tolerance
  }

  public double getTX() {
    return m_TX;
  }

  private double getDistanceToTarget() {
    if (this.m_HasTarget) {
      // Calculate horizontal distance using trigonometry.
      // All heights are now standardized to METERS in VisionConstants.
      double angleToTarget = Math.toRadians(mountAngle.get() + m_TY);
      
      // Safety check: Ensure we are not dividing by zero or a very small number
      if (Math.abs(Math.tan(angleToTarget)) < 0.01) return 0.0;

      double distanceMeters = (VisionConstants.kAprilTagHeight - VisionConstants.kCameraHeight) /
          Math.tan(angleToTarget);

      // Safety check: Ensure distance is positive (target must be above/in front)
      return Math.max(0.0, distanceMeters);
    } else {
      return 0.0;
    }
  }

  private double calculateSpeedFromDistance(double distance) {
    // If we have no target, or distance is invalid, return idle speed
    if (distance <= 0.0) {
      Telemetry.putString("Launcher/Shot Status", "NO TARGET");
      return LauncherConstants.kLauncherIdleSpeed;
    }

    // Check for range limits based on the shot map (24 inches to 96 inches)
    double minMeters = Units.inchesToMeters(23.0);
    double maxMeters = Units.inchesToMeters(97.0);

    if (distance < minMeters) {
      Telemetry.putString("Launcher/Shot Status", "TOO CLOSE");
      return LauncherConstants.kLauncherIdleSpeed;
    }
    if (distance > maxMeters) {
      Telemetry.putString("Launcher/Shot Status", "TOO FAR");
      return LauncherConstants.kLauncherIdleSpeed;
    }

    Telemetry.putString("Launcher/Shot Status", "READY");
    // kShotMap uses meters as its lookup key
    return LauncherConstants.kShotMap.get(distance) + shotOffset.get();
  }

  public Command align(DoubleSupplier xSpeed, DoubleSupplier ySpeed, java.util.function.BooleanSupplier fieldRelative) {
    return Commands.run(() -> {
      double rotationSpeed = 0;
      if (m_HasTarget) {
        rotationSpeed = m_AlignPID.calculate(m_TX, 0);

        // Only apply feed-forward when far from setpoint to overcome static friction
        if (!m_AlignPID.atSetpoint()) {
          double minStep = 0.05;
          // Only add feed-forward if PID output is too small to overcome friction
          if (Math.abs(rotationSpeed) < minStep) {
            rotationSpeed = Math.copySign(minStep, rotationSpeed);
          }
        }

        rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
      }

      m_DriveSystem.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotationSpeed, fieldRelative.getAsBoolean());
    }, m_DriveSystem).withName("AlignToTargetPID");
}

  public Command launchWithVision() {
    return this.run(() -> {
      double distance = getDistanceToTarget();
      this.m_TargetSpeed = this.calculateSpeedFromDistance(distance);
      this.runMotor(this.m_TargetSpeed);
    })
        .withName("LaunchWithVision")
        .finallyDo((interrupted) -> {
          if (this.m_IdleOn) {
            // Return to the "Idle" state if that's where we started
            this.runMotor(LauncherConstants.kLauncherIdleSpeed);
            this.m_TargetSpeed = LauncherConstants.kLauncherIdleSpeed;
          } else {
            // Otherwise, shut it down
            this.stopMotor();
            this.m_TargetSpeed = 0.0;
          }
        });
  }

  public Command launchWithJoystick(DoubleSupplier supplierSpeed) {
    return this.run(() -> {
      Telemetry.putNumber("Launcher/Slider Value", supplierSpeed.getAsDouble());
      double speed = (supplierSpeed.getAsDouble() + 1) / 2;
      
      this.m_TargetSpeed = this.calculateRPMFromSlider(speed);
      this.runMotor(this.m_TargetSpeed);
    })
        .withName("LaunchWithJoystick")
        .finallyDo((interrupted) -> {
          if (this.m_IdleOn) {
            // Return to the "Idle" state if that's where we started
            this.runMotor(LauncherConstants.kLauncherIdleSpeed);
            this.m_TargetSpeed = LauncherConstants.kLauncherIdleSpeed;
          } else {
            // Otherwise, shut it down
            this.stopMotor();
            this.m_TargetSpeed = 0.0;
          }
        });
  }

  public Command idle() {
    return this.run(() -> {
      this.m_TargetSpeed = LauncherConstants.kLauncherIdleSpeed;
      this.runMotor(this.m_TargetSpeed);
      this.m_IdleOn = true; // Mark that we are now in "Idle Mode"
    })
        .withName("LauncherIdle");
  }

  public Command off() {
    return this.runOnce(() -> {
      this.stopMotor();
      this.m_TargetSpeed = 0.0;
      this.m_IdleOn = false; // Turn off the "Return to Idle" behavior
    })
        .withName("LauncherOff");
  }

  public TalonFX getMotor() {
    return this.m_IntakeMotor;
  }
}
