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
        // If any status isn't "OK", the master light goes red
        allSystemsNominal = motorStatus.values().stream().allMatch(s -> s.equals("OK"));
    }

    public static String getReport() {
        return motorStatus.toString(); // Or something prettier for Elastic
    }

    public static boolean isHealthy(String name) {
        String status = motorStatus.get(name);
        // If we haven't checked it yet, assume OK to avoid blocking initial actions,
        // or return true only if explicitly "OK".
        return status == null || status.equals("OK");
    }

    public static boolean isEverythingOk() {
        return allSystemsNominal;
    }
}
