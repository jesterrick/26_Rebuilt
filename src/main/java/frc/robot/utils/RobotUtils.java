// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

public class RobotUtils {

    /**
     * Calculates the position conversion factor for linear mechanisms.
     * Result is Inches per 1 Motor Rotation.
     */
    public static double calculateLinearFactor(double gearRatio, double drumDiameter) {
        // Circumference (PI * D) divided by Gear Ratio
        return (Math.PI * inchesToMeters(drumDiameter)) / gearRatio;
    }

    /**
     * Converts a Position Factor to a Velocity Factor.
     * Result is Units per Second (e.g., Inches/Sec).
     */
    public static double toVelocityPerSecond(double positionFactor) {
        return positionFactor / 60.0;
    }

    /**
     * Calculates the distance traveled given a velocity and time.
     * Result is in the same unit as velocity * time.
     * @param velocity The velocity of the object.
     * @param time The time duration.
     * @return The distance traveled.
     */
    public static double calculateDistance(double velocity, double time) {
        return velocity * time;
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }
}
