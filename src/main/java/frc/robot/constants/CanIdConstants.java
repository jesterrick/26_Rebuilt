package frc.robot.constants;

/**
 * The CanIdConstants class centralizes all CAN IDs for the robot's motors,
 * ensuring easy management and modification of hardware addresses.
 */

/*
 * CAN ID RANGES
 * Turning 1 - 4
 * Driving 5 - 8
 * Intake 10 - 14
 * Rollers 15 - 19
 * Extender 20 - 24
 * Climber 25-29
 * Feeder 30 - 34
 * Launcher 35-39
 * PDH - 60
 */


/*
 * CAN ID RANGES
 * Turning 1 - 4
 * Driving 5 - 8
 * Intake 10 - 14
 * Rollers 15 - 19
 * Extender 20 - 24
 * Climber 25-29
 * Feeder 30 - 34
 * Launcher 35-39
 * PDH - 60
 */

public class CanIdConstants {
    // CAN IDs for the Swerve Modules
    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;;

    // CAN IDs for the Extender
    public static final int kExtenderMotor1 = 20;
    public static final int kExtenderMotor2 = 21;

    // CAN IDs for Climber
    public static final int kClimberMotor1 = 25;
    public static final int kClimberMotor2 = 26;

    // CAN IDs for the Intake
    // CAN IDs for the Intake Rotator
    public static final int kIntakeMotor = 10;
    public static final int kIntakeRaiseMotor = 11;

    // CAN IDs for the Rollers
    public static final int kRollerMotor = 15;

    // CAN IDs for the Feeder
    public static final int kFeederMotor = 30;

    // CAN IDs for the Launcher
    public static final int kLauncherMotor = 35;
    //public static final int kLauncherFollowMotor = 36;
}
