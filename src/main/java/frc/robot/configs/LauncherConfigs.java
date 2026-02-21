// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.LauncherConstants;

/** Add your docs here. */
public class LauncherConfigs {
  public static final TalonFXConfiguration config = new TalonFXConfiguration();

  static {

    var slot0 = config.Slot0;
    slot0.kP = LauncherConstants.kP;
    slot0.kI = LauncherConstants.kI;
    slot0.kD = LauncherConstants.kD;
    slot0.kV = LauncherConstants.kV;
    slot0.kS = LauncherConstants.kS;
    slot0.kA = LauncherConstants.kA;

    config.Feedback.SensorToMechanismRatio = LauncherConstants.kVelocityFactor;
    
    config.CurrentLimits.SupplyCurrentLimit = LauncherConstants.kCurrentLimit;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }
}
