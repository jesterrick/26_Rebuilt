package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * The VisionConstants class stores all constant values related to the robot's
 * vision system.
 * This includes configurations for Limelight cameras, such as network table
 * keys,
 * pipeline IDs, and default values for vision-based calculations.
 */
public class VisionConstants {
  /** The team number for the FRC team. */
  public static final int TEAM_NUMBER = 5919;
  /** The NetworkTables name for the Limelight camera. */
  public static final String LIMELIGHT_NAME = "limelight";
  /** The URL for accessing the Limelight's web interface. */
  public static final String LIMELIGHT_URL = "http://limelight.local:5800";

  /** The pipeline ID for driver camera feed. */
  public static final int DRIVER_PIPELINE = 0;
  /** The pipeline ID for vision processing (e.g., target tracking). */
  public static final int VISION_PIPELINE = 1;

  /** NetworkTables key for checking if a valid target is present (`tv`). */
  public static final String kTargetValidKey = "tv";
  /** NetworkTables key for retrieving the target ID (`tid`). */
  public static final String kTargetIdKey = "tid";
  /**
   * NetworkTables key for retrieving the vertical offset of the target (`ty`).
   */
  public static final String kTargetYKey = "ty";
  public static final String kTargetXKey = "tx";
  
  /** Default value for `kTargetValidKey` if no target is found. */
  public static final double kDefaultTargetValid = 0.0;
  /** Default value for `kTargetIdKey` if no target is found. */
  public static final int kDefaultTargetId = -1;
  /** Default value for `kTargetYKey` if no target is found. */
  public static final double kDefaultTargetY = 0.0;
  public static final double kDefaultTargetX = 0.0;

  // values of the field components for the limelight calculations
  /** The height of the target on the field, in meters. */
  public static final double kTargetHeight = Units.inchesToMeters(72.0);
  /** The height of the AprilTag on the field, in meters. */
  public static final double kAprilTagHeight = Units.inchesToMeters(44.25);
  /** The height of the Limelight camera from the ground, in meters. */
  public static final double kCameraHeight = Units.inchesToMeters(19.25);
  /** The mounting angle of the Limelight camera, in degrees. */
  public static final double kMountAngle = 31.5;
}
