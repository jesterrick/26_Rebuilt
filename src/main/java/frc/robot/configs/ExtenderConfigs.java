// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;

import frc.robot.constants.ExtenderConstants;
import frc.robot.constants.GlobalConstants;

/** Add your docs here. */
public class ExtenderConfigs {
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config.encoder
        .positionConversionFactor(ExtenderConstants.kPositionFactor)
        .velocityConversionFactor(ExtenderConstants.kVelocityFactor);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ExtenderConstants.kP, ExtenderConstants.kI, ExtenderConstants.kD)
        .allowedClosedLoopError(ExtenderConstants.kPositionTolerance, ClosedLoopSlot.kSlot0);

    config.closedLoop.feedForward
        .kV(ExtenderConstants.kV)
        .kS(ExtenderConstants.kS)
        .kA(ExtenderConstants.kA);

    config.closedLoop.maxMotion
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .cruiseVelocity(ExtenderConstants.kCruiseVelocity)
        .maxAcceleration(ExtenderConstants.kMaxAccel)
        .allowedProfileError(ExtenderConstants.kAllowedError);

    config.softLimit
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(ExtenderConstants.kExtendOutTarget)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(0.0);

    config
        .voltageCompensation(GlobalConstants.kMedVoltageCompensation)
        .smartCurrentLimit(ExtenderConstants.kCurrentLimit);

    config.idleMode(IdleMode.kCoast);
  }
}
