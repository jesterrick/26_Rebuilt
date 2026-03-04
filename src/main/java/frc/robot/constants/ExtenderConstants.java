package frc.robot.constants;

import frc.robot.utils.RobotUtils;

public class ExtenderConstants {

  /* Motor Control Configs */
  public static final double kMotorSpeed = 1.0;
  public static final double kP = 0.8;
  public static final double kI = 0.00;
  public static final double kD = 0.0;
  public static final double kV = 0.8;
  public static final double kS = 0.6;
  public static final double kA = 0.4;
  public static final double kMaxAccel = 12.0;

  public static final double kAllowedError = RobotUtils.inchesToMeters(0.5);

  public static final double kMaxPositionDifference = RobotUtils.inchesToMeters(2.0);

  public static final double kExtendOutTarget = RobotUtils.inchesToMeters(12.0);

  public static final double kErrorToleranceInInches = 0.1;

  /* Fixed Hardware Configs */

  public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;

  public static final double kGearRatio = 36.0;

  private static final double pitchDiameter = 2.10;

  public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, pitchDiameter);;
  public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

  public static final double kPositionTolerance = RobotUtils.inchesToMeters(kErrorToleranceInInches); // Your "close enough" value
  //public static final double kCruiseVelocity = NeoMotorConstants.kFreeSpeedRpm * kMotorSpeed * kVelocityFactor;
  public static final double kCruiseVelocity = 1.5;
}
