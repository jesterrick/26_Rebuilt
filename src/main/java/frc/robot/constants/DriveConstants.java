package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAutonomousSpeed = 3.0;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Front Left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Front Right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Rear Left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Rear Right

    public static final double kDriveBaseRadius = Math.sqrt(Math.pow(kWheelBase / 2, 2) + Math.pow(kTrackWidth / 2, 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = true;

    public static final double kDriveP = 0.4;
    public static final double kDriveI = 0.00;
    public static final double kDriveD = 0.00;
    public static final double kDriveV = 0.2;
    public static final double kDriveS = 0.03;

    public static final double kTurnP = 0.75;
    public static final double kTurnI = 0.00;
    public static final double kTurnD = 0.00;
    public static final double kTurnV = 0.2;
    public static final double kTurnS = 0.1;
}