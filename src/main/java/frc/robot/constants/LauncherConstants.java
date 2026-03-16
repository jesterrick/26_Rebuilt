package frc.robot.constants;

import frc.robot.utils.RobotUtils;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

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

  public static final double kLauncherIdleSpeed = 10.0; // RPS
  public static final double kMinLaunchSpeed = 45.0; // RPS
  public static final double kMaxLaunchSpeed = 75.0; // RPS
  public static final double kTolerance = 3.0; // RPS tolerance

  public static final double kAutoLaunchSpeed = 50.0; // RPS

  public static final double kAlignP = 0.015;
  public static final double kAlignD = 0.01;
  public static final double kAlignTolerace = 3.0;
  public static final double kShotOffset = 0.0;

  /* Fixed Hardware Configs */

  private static final double wheelSize = 4.0;
  public static final double kGearRatio = 1.0;

  public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, wheelSize);
  public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

  public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;

  public static final InterpolatingDoubleTreeMap kShotMap = new InterpolatingDoubleTreeMap();

  static {
    // format: kShotMap.put(distanceInMeters, speedInRPS);
    kShotMap.put(Units.inchesToMeters(24.0), 45.0);
    kShotMap.put(Units.inchesToMeters(48.0), 51.0);
    kShotMap.put(Units.inchesToMeters(67.0), 57.0);
    kShotMap.put(Units.inchesToMeters(96.0), 65.0);
  }

}
