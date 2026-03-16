// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

/**
 * Centralized utility for alliance detection and caching.
 * Prevents redundant calls to DriverStation and handles defaulting.
 */
public class AllianceUtils {
    private static Optional<DriverStation.Alliance> m_cachedAlliance = Optional.empty();

    /**
     * Updates the cached alliance from the DriverStation.
     * Call this in robotPeriodic() or disabledPeriodic().
     */
    public static void update() {
        if (m_cachedAlliance.isEmpty() || edu.wpi.first.wpilibj.RobotBase.isSimulation()) {
            Optional<DriverStation.Alliance> currentAlliance = DriverStation.getAlliance();
            
            // Only update if we have a value and it's different from what we had
            if (currentAlliance.isPresent() && !currentAlliance.equals(m_cachedAlliance)) {
                m_cachedAlliance = currentAlliance;
                SmartDashboard.putString("Robot/Alliance", m_cachedAlliance.get().name());
            }
        }
    }

    /**
     * Returns the cached alliance or defaults to Blue if unknown.
     * @return The current alliance.
     */
    public static DriverStation.Alliance getAlliance() {
        return m_cachedAlliance.orElse(DriverStation.Alliance.Blue);
    }

    /**
     * @return True if the robot is on the Red alliance.
     */
    public static boolean isRed() {
        return getAlliance() == DriverStation.Alliance.Red;
    }

    /**
     * @return True if the alliance has been successfully detected.
     */
    public static boolean isDetected() {
        return m_cachedAlliance.isPresent();
    }
}
