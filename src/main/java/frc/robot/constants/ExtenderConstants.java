package frc.robot.constants;

import frc.robot.utils.RobotUtils;

public class ExtenderConstants {

  /* Motor Control Configs */
  public static final double kMotorSpeed = 0.15;
  public static final double kP = 0.05;
  public static final double kI = 0.00;
  public static final double kD = 0.05;
  public static final double kV = 0.05;
  public static final double kS = 0.15;
  public static final double kA = 0.05;
  public static final double kMaxAccel = 7.0;

  public static final double kAllowedError = RobotUtils.inchesToMeters(0.1);

  public static final double kMaxPositionDifference = RobotUtils.inchesToMeters(1);

  public static final double kExtendOutTarget = RobotUtils.inchesToMeters(11.3);

  public static final double kErrorToleranceInInches = 0.2;

  /* Fixed Hardware Configs */

  public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;

  public static final double kGearRatio = 1.0;

  private static final double pitchDiameter = 2.14;

  public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, pitchDiameter);;
  public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

  public static final double kPositionTolerance = RobotUtils.inchesToTicks(kErrorToleranceInInches, pitchDiameter, kGearRatio); // Your "close enough" value
  public static final double kCruiseVelocity = NeoMotorConstants.kFreeSpeedRpm * kMotorSpeed * kVelocityFactor;
}
