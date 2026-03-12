// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.concurrent.ConcurrentHashMap;

/** Add your docs here. */
public class RobotHealth {
    // Thread-safe map to store status. Think of it as a scoreboard.
    private static final ConcurrentHashMap<String, String> motorStatus = new ConcurrentHashMap<>();
    private static boolean allSystemsNominal = true;

    public static void updateStatus(String name, String status) {
        motorStatus.put(name, status);
        // All systems nominal if everything is "OK"
        // Warnings are allowed, but the "Master Good" light only goes off for ERRORS
        allSystemsNominal = motorStatus.values().stream().noneMatch(s -> s.contains("ERROR"));
    }

    public static boolean isHealthy(String name) {
        String status = motorStatus.get(name);
        // Returns true if status is either OK or just a WARNING.
        // We only block commands for "ERROR" status.
        return status == null || !status.contains("ERROR");
    }

    public static boolean hasWarning(String name) {
        String status = motorStatus.get(name);
        return status != null && status.contains("WARNING");
    }

    public static boolean isEverythingOk() {
        return allSystemsNominal;
    }
}
