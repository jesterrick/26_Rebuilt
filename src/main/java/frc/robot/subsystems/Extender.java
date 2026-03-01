// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.configs.ExtenderConfigs;
import frc.robot.constants.CanIdConstants;
import frc.robot.constants.ExtenderConstants;
import frc.robot.utils.RobotUtils;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase {
  private final SparkMax m_LeaderMotor;
  private final SparkMax m_FollowMotor;

  private final SparkClosedLoopController m_LeaderController;

  private final RelativeEncoder m_LeaderEncoder;
  private final RelativeEncoder m_FollowEncoder;

  private double m_TargetPOS = 0.0;
  private boolean m_isFaulted = false;


  /** Creates a new Extender. */
  public Extender() {

    this.m_LeaderMotor = new SparkMax(CanIdConstants.kExtenderMotor1, SparkLowLevel.MotorType.kBrushless);
    this.m_FollowMotor = new SparkMax(CanIdConstants.kExtenderMotor2, SparkLowLevel.MotorType.kBrushless);

    this.m_LeaderController = this.m_LeaderMotor.getClosedLoopController();

    this.m_LeaderEncoder = this.m_LeaderMotor.getEncoder();
    this.m_FollowEncoder = this.m_FollowMotor.getEncoder();

    // Configure motor controllers (no encoder config needed for SparkMax)
    SparkMaxConfig leaderConfig = new SparkMaxConfig().apply(ExtenderConfigs.config);
    leaderConfig.inverted(true);
    this.m_LeaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig().apply(ExtenderConfigs.config);
    followerConfig.follow(this.m_LeaderMotor, true);
    this.m_FollowMotor.configure(followerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.m_LeaderEncoder.setPosition(0);
    this.m_FollowEncoder.setPosition(0);    
  }

  @Override
  public void periodic() {
    if (m_isFaulted) {
      stop(); // Keep them stopped if we are already in a fault state
      return;
    }

    double leaderPos = this.m_LeaderEncoder.getPosition();
    double followPos = this.m_FollowEncoder.getPosition();

    SmartDashboard.putNumber("Extender/Leader POS", RobotUtils.metersToInches(leaderPos));
    SmartDashboard.putNumber("Extender/Follow POS", RobotUtils.metersToInches(followPos));
    SmartDashboard.putNumber("Extender/Target POS", RobotUtils.metersToInches(this.m_TargetPOS));
    SmartDashboard.putNumber("Extender/Leader Current", m_FollowMotor.getOutputCurrent());
    SmartDashboard.putNumber("Extender/Follow Current", m_FollowMotor.getOutputCurrent());

    if (Math.abs(leaderPos - followPos) > ExtenderConstants.kMaxPositionDifference) {
      m_isFaulted = true; // Trip the software breaker
      stop();
    }
  }

  private void stop() {
    this.m_LeaderMotor.stopMotor();
    this.m_FollowMotor.stopMotor();
  }

  private Command goToPosition(double position) {
    return this.run(() -> {
      if (!m_isFaulted) {
        this.m_LeaderController.setSetpoint(position, ControlType.kMAXMotionPositionControl);
        this.m_TargetPOS = position;
      }
    })
        .until(() -> m_isFaulted ||
          Math.abs(m_LeaderEncoder.getPosition() - position) < ExtenderConstants.kPositionTolerance)
        .withTimeout(3.0)
        .finallyDo((interrupted) -> stop())
        .withName("ExtenderTo" + position);
  }

  public Command stow() {
    return goToPosition(0.0)
        .withName("ExtenderIn");
  }

  public Command fullExtend() {
    return goToPosition(ExtenderConstants.kExtendOutTarget)
        .withName("ExtenderOut");
  }

  public SparkMax getLeader() {
    return m_LeaderMotor;
  }

  public SparkMax getFollower() {
    return m_FollowMotor;
  }
}
