// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Centralized telemetry management. 
 * Allows for easy toggling of verbose debug data to save CAN/Network bandwidth.
 */
public class Telemetry {
    private static boolean m_debugEnabled = true;

    public static void setDebugEnabled(boolean enabled) {
        m_debugEnabled = enabled;
    }

    public static void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void putString(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    /** Only logs if debug mode is enabled. Use for high-frequency or verbose data. */
    public static void putDebugNumber(String key, double value) {
        if (m_debugEnabled) {
            SmartDashboard.putNumber("Debug/" + key, value);
        }
    }
}
