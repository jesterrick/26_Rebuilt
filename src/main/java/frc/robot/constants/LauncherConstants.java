package frc.robot.constants;

import frc.robot.utils.RobotUtils;

public class LauncherConstants {

  /* Motor Control Configs */

  public static final double kP = 0.1;
  public static final double kI = 0.00;
  public static final double kD = 0.00;
  public static final double kV = 0.00;
  public static final double kS = 0.1;
  public static final double kA = 0.05;
  public static final double kMaxAccel = 25.0;

  public static final double kAllowedError = 0.1;

  public static final double kLauncherIdleSpeed = 500;

  public static final double kMinLaunchSpeed = 1500;
  public static final double kMaxLaunchSpeed = 4500;

  public static final double kTolerance = 200.0;

  /* Fixed Hardware Configs */

  private static final double wheelSize = 4.0;
  private static final double kGearRatio = 1.0;

  public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, wheelSize);
  public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

  public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;

}
