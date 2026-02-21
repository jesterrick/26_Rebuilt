// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.LauncherConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  private final TalonFX m_LeaderMotor;
  private final TalonFX m_FollowMotor;
  private final Follower m_Follower;
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);

  private double m_TargetSpeed;

  /** Creates a new Launcher. */
  public Launcher() {
    this.m_LeaderMotor = new TalonFX(CanIdConstants.kLauncherLeaderMotor);
    this.m_FollowMotor = new TalonFX(CanIdConstants.kLauncherFollowMotor);
    this.m_LeaderMotor.getConfigurator().apply(LauncherConfigs.config);

    this.m_Follower = new Follower(this.m_LeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    this.m_FollowMotor.setControl(this.m_Follower);

    this.m_TargetSpeed = 0.0;
  }

  @Override
  public void periodic() {

  }

  private double calculateRPMFromSlider(double slideValue) {
    double min = LauncherConstants.kMinLaunchSpeed;
    double max = LauncherConstants.kMaxLaunchSpeed;

    double speed = slideValue * ((max - min) * slideValue);
    return speed;
  }

  private void runMotor(double speed) {
    this.m_LeaderMotor.setControl(m_dutyCycleOut.withOutput(speed));
  }

  private void stopMotor() {
    this.m_LeaderMotor.stopMotor();
    this.m_FollowMotor.stopMotor();
  }

  private double getActualVelocity() {
    return this.m_LeaderMotor.getVelocity().getValueAsDouble();
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
      double speed = (speedSupplier.getAsDouble() + 1) / 2;
      this.runMotor(this.calculateRPMFromSlider(speed));
    })
        .withName("LauncherOn")
        .finallyDo(this::stopMotor); // Replaces the end() method in your old file
  }

  public Command idle() {
    return this.run(() -> this.runMotor(LauncherConstants.kLauncherIdleSpeed))
        .withName("LauncherIdle")
        .finallyDo(this::stopMotor);
  }

  public Command off() {
    return this.run(() -> this.stopMotor());
  }

  public TalonFX getLeader(){
    return this.m_LeaderMotor;
  }

  public TalonFX getFollower(){
    return this.m_FollowMotor;
  }
}
