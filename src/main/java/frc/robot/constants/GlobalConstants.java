// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class GlobalConstants {
  /** High current limit, typically used for high-power mechanisms like climbers or heavy lifting systems. */
    public static final int kHighCurrentLimit = 60; 
    /** Medium current limit, often used for drive motors or heavy shooters. */
    public static final int kMediumCurrentLimit = 40; 
    /** Low current limit, suitable for lighter mechanisms such as intakes, small extenders, or turning motors. */
    public static final int kLowCurrentLimit = 20; 

    public static final double kLowVoltageCompensation = 10.0;
    public static final double kMedVoltageCompensation = 11.0;
    public static final double kHighVoltageCompensation = 12.0;
}
