// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.FeederConstants;

/** Add your docs here. */
public class FeederConfigs {
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config.encoder
      .positionConversionFactor(FeederConstants.kPositionFactor)
      .velocityConversionFactor(FeederConstants.kVelocityFactor);

    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD);
 
        config.closedLoop.feedForward
        .kV(FeederConstants.kV)
        .kS(FeederConstants.kS)
        .kA(FeederConstants.kA);

    config.smartCurrentLimit(FeederConstants.kCurrentLimit);

    config.idleMode(IdleMode.kBrake);
  }
}
