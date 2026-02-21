package frc.robot.constants;

import frc.robot.utils.RobotUtils;

public class ExtenderConstants {

  /* Motor Control Configs */
  public static final double kMotorSpeed = 0.5;
  public static final double kP = 0.1;
  public static final double kI = 0.00;
  public static final double kD = 0.00;
  public static final double kV = 0.00;
  public static final double kS = 0.2;
  public static final double kA = 0.05;
  public static final double kMaxAccel = 25.0;

  public static final double kAllowedError = 0.1;

  public static final double kMaxPositionDifference = 0.5;

  public static final double kExtendOutTarget = 11.5;

  /* Fixed Hardware Configs */

  public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;

  public static final double kGearRatio = 1.0;

  private static final double pitchDiameter = 1.25;

  public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, pitchDiameter);;
  public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

  public static final double kCruiseVelocity = NeoMotorConstants.kFreeSpeedRpm * kMotorSpeed * kVelocityFactor;
}
