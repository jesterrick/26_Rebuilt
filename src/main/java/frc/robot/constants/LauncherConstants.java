package frc.robot.constants;

import frc.robot.utils.RobotUtils;

public class LauncherConstants {

  /* Motor Control Configs */

  public static final double kP = 0.11;
  public static final double kI = 0.00;
  public static final double kD = 0.01;
  public static final double kV = 0.12;
  public static final double kS = 0.15;
  public static final double kA = 0.0;
  public static final double kMaxAccel = 25.0;

  public static final double kAllowedError = 0.1;

  public static final double kLauncherIdleSpeed = 8.0; // ~480 RPM
  public static final double kMinLaunchSpeed = 25.0; // 1500 RPM
  public static final double kMaxLaunchSpeed = 55.0; // 4500 RPM (need to change)
  public static final double kTolerance = 3.0; // ~180 RPM tolerance

  /* Fixed Hardware Configs */

  private static final double wheelSize = 4.0;
  public static final double kGearRatio = 1.0;

  public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, wheelSize);
  public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

  public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;

}
