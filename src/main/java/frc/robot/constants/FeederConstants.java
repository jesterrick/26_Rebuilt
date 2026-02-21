package frc.robot.constants;

import frc.robot.utils.RobotUtils;

public class FeederConstants {

    /* Motor Control Configs */
    public static final double kMotorSpeed = 0.5;
    public static final double kP = 0.1;
    public static final double kI = 0.00;
    public static final double kD = 0.00;
    public static final double kV = 0.00;
    public static final double kS = 0.1;
    public static final double kA = 0.05;

    /* Fixed Hardware Configs */

    private static final double wheelSize = 4.0;
    private static final double kGearRatio = 1.0;    

    public static final double kPositionFactor = RobotUtils.calculateLinearFactor(kGearRatio, wheelSize);
    public static final double kVelocityFactor = RobotUtils.toVelocityPerSecond(kPositionFactor);

    public static final int kCurrentLimit = GlobalConstants.kMediumCurrentLimit;
}
