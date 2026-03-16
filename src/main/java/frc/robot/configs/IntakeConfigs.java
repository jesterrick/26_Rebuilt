// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IntakeConstants;

/** Add your docs here. */
public class IntakeConfigs {
  public static final SparkMaxConfig config = new SparkMaxConfig();

  static {
    config.encoder
      .positionConversionFactor(IntakeConstants.kPositionFactor)
      .velocityConversionFactor(IntakeConstants.kVelocityFactor);
 
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
 
        config.closedLoop.feedForward
        .kV(IntakeConstants.kV)
        .kS(IntakeConstants.kS)
        .kA(IntakeConstants.kA);

    config.inverted(true);

    config
      .voltageCompensation(GlobalConstants.kLowVoltageCompensation)
      .smartCurrentLimit(IntakeConstants.kCurrentLimit);

    config.idleMode(IdleMode.kBrake);
  }
}
