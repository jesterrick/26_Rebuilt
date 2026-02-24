// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.LauncherConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  private final TalonFX m_IntakeMotor;
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

  private double m_TargetSpeed;
  private boolean idleOn = false;

  /** Creates a new Launcher. */
  public Launcher() {
    this.m_IntakeMotor = new TalonFX(CanIdConstants.kLauncherMotor);
    this.m_IntakeMotor.getConfigurator().apply(LauncherConfigs.config);

    this.m_TargetSpeed = 0.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Launcher/Current Speed", getActualVelocity());
    SmartDashboard.putNumber("Launcher/Target Speed", m_TargetSpeed);
    SmartDashboard.putBoolean("Launcher/At Speed", atSpeed());
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

  public Command launch(DoubleSupplier speedSupplier) {
    return this.run(() -> {
      SmartDashboard.putNumber("Launcher/Slider Value", speedSupplier.getAsDouble());
      double speed = (speedSupplier.getAsDouble() + 1) / 2;
      this.m_TargetSpeed = this.calculateRPMFromSlider(speed);
      this.runMotor(this.m_TargetSpeed);
    })
        .withName("LauncherOn")
        .finallyDo((interrupted) -> {
          if (this.idleOn) {
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
      this.idleOn = true; // Mark that we are now in "Idle Mode"
    })
        .withName("LauncherIdle");
  }

  public Command off() {
    return this.runOnce(() -> {
      this.stopMotor();
      this.m_TargetSpeed = 0.0;
      this.idleOn = false; // Turn off the "Return to Idle" behavior
    })
        .withName("LauncherOff");
  }

  public TalonFX getMotor() {
    return this.m_IntakeMotor;
  }
}
