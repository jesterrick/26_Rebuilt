package frc.robot.constants;

/**
 * The OIConstants class stores all constant values related to the Operator Interface (OI).
 * This includes joystick port assignments and button mappings for various robot actions,
 * providing a centralized configuration for robot controls.
 */
public class OIConstants {
    /** The USB port for the driver's joystick. */
    public static final int kDriverJoystickPort = 0;
    /** The USB port for the operator's joystick. */
    public static final int kOperatorJoystickPort = 1;
    
    /** The deadband for joystick inputs, below which input is ignored to prevent unintended movement. */
    public static final double kDriveDeadband = 0.1;

    /** DRIVER BUTTONS */
    public static final int kOrientRobot = 1;
    public static final int kAlignRobot = 2;
    public static final int kToggleDriveMode = 5;

    /** OPERATOR BUTTONS */
    /** Button ID for the intake mechanism to receive game pieces. */
    public static final int kIntakeReceiveButton = 4;
    /** Button ID for the intake mechanism to eject game pieces. */
    public static final int kIntakeEjectButton = 6;
    /** Button ID for launching game pieces. */
    public static final int kLauncherButton = 18;
    /** Button ID for extending a mechanism outwards. */
    public static final int kExtenderOutButton = 1;
    /** Button ID for retracting a mechanism inwards. */
    public static final int kExtenderInButton = 2;

    public static final int kExtenderFollowJoystick = 3;
    
    public static final int kSwitchLaunchMode = 5; 

    /** Button ID to home or reset the extender mechanism. */
    public static final int kExtenderHomeButton = 8;

    public static final int kEmergencyStop = 14;
    public static final int kClearExtenderFaults = 13;

    /** Button ID to activate the launcher's idle speed. */
    public static final int kLauncherIdleOnButton = 16;
    /** Button ID to deactivate the launcher's idle speed. */
    public static final int kLauncherIdleOffButton = 17;
    /** Button ID to extend the climber mechanism. */
    public static final int kClimberExtendButton = 1;
    /** Button ID to retract the climber mechanism. */
    public static final int kClimberRetractButton = 2;
    /** Button ID to home or reset the climber mechanism. */
    public static final int kClimberHomeButton = 8;

    /** Slider ID to manually adjust launcher speed */
    public static final int kManualLaunchSlider = 5;
    
}
